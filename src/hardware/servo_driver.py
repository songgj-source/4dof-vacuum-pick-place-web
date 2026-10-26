"""Disarmed-by-default actuator core. Only explicit arm can write goals/torque."""
import copy
import threading
import time
from hardware.servo_mapping import finite
from motion import guard_mode


def reversal_filtered(raw, last, direction):
    """Suppress a lone 1-pulse step against the last commanded direction.

    IK sample noise is about one pulse; following it makes the servo reverse
    direction across its backlash/deadzone every tick (visible as shaking).
    A real reversal continues beyond one pulse and is passed through.
    """
    step = raw-last
    if abs(step) == 1 and direction and step == -direction:
        return last
    return raw


class ServoDriver:
    def __init__(self, bus, profile, sleep=time.sleep, clock=time.monotonic):
        self.bus, self.profile = bus, profile
        self.sleep, self.clock = sleep, clock
        self.state, self.commissioning = 'disarmed', False
        self.last_fault, self.hold_confirmed = None, False
        self.lock = threading.RLock()
        self.cache_lock = threading.Lock()
        # 인계 때 목표–위치 차이 허용(펄스)과 하중 처짐 보정 한도. 기본값이 집기 동작의 안전 기준이다.
        # HOME 복귀(return_home.py)만 2026-10-21 사용자 지시로 둘 다 풀어 쓴다(멈춰 있으면 처짐 크기와 상관없이 인계).
        self.max_pending_pulses, self.max_load_offset = 8, 6
        self.feedback_cache = None
        self.hold_results, self.hold_error = {}, None
        self.hold_targets = {}
        self.hold_offsets = {}   # 관절별 (위치-쓴 목표) 보상값. 확인된 hold에서만 의미가 있다.
        self.closed, self.shutdown_error = False, None
        self.last_probe = None
        self.names = tuple(j.name for j in profile.joints)

    def _snapshot(self, torque=None, fault_on_error=True, try_hold=True):
        # Even adapter/preflight reads share the same lock as every write/close.
        with self.lock:
            if self.closed: raise RuntimeError('driver transport is closed')
            try:
                data = self._read_snapshot(torque)
            except BaseException as exc:
                with self.cache_lock: self.feedback_cache = None
                if fault_on_error and self.state in ('armed','hold_only'):
                    self._fault(exc,try_hold=try_hold)
                raise
            with self.cache_lock: self.feedback_cache = copy.deepcopy(data)
            return data

    def _read_snapshot(self, torque):
        self.profile.check_sources()
        data = self.bus.read()
        age = self.clock()-finite(data['timestamp'],'feedback timestamp')
        if not 0 <= age <= .5 or set(data['joints']) != set(self.names):
            raise ValueError('stale/incomplete feedback')
        for joint in self.profile.joints:
            row = data['joints'][joint.name]
            if row['id'] != joint.motor_id or row['mode'] != 0:
                raise ValueError('ID/mode mismatch')
            for field,expected in self.profile.data['expected_registers'][joint.name].items():
                if row[field] != expected: raise ValueError(f'{joint.name}: {field} changed')
            raw = row['position']
            if type(raw) is not int or not joint.raw_min <= raw <= joint.raw_max:
                raise ValueError('position outside controller/profile range')
            if row['torque'] not in (0,1) or torque is not None and row['torque'] != torque:
                raise ValueError('unexpected torque state')
        return data

    def _describe(self, data):
        data['driver_state'] = self.state
        data['hold_confirmed'] = self.hold_confirmed
        data['hold_results'] = copy.deepcopy(self.hold_results)
        data['last_fault'], data['hold_error'] = self.last_fault, self.hold_error
        for joint in self.profile.joints:
            row = data['joints'][joint.name]
            try: row['radians_candidate'] = joint.raw_to_rad(row['position'])
            except ValueError as exc: row['mapping_error'] = str(exc)
        return data

    def cached_feedback(self):
        """Fresh immutable copy; never waits for the execution/I/O lock."""
        with self.cache_lock: data = copy.deepcopy(self.feedback_cache)
        if data is None or not 0 <= self.clock()-data['timestamp'] <= .5:
            raise ValueError('execution feedback absent/stale')
        return self._describe(data)

    def inspect(self):
        with self.lock:
            # Explicit inspection remains write-free, even on an armed fault.
            return self._describe(self._snapshot(
                torque=1 if self.state in ('armed','hold_only') else None,try_hold=False))

    def monitor(self):
        """ROS armed monitoring may attempt a fault hold; never arms/disarms."""
        with self.lock:
            return self._describe(self._snapshot(torque=1 if self.state in ('armed','hold_only') else None))

    def dry_run(self, joint, delta):
        with self.lock:
            snapshot = self._snapshot(try_hold=False)
            return self._relative_target(snapshot,joint,delta)

    def _relative_target(self, snapshot, name, delta):
        if name not in self.names or type(delta) is not int or not 0 < abs(delta) <= 8:
            raise ValueError('commissioning: one joint, integer delta of 1–8 pulses required')
        joint = next(j for j in self.profile.joints if j.name == name)
        start = snapshot['joints'][name]['position']
        target = start+delta
        if not joint.raw_min+16 <= min(start,target) <= max(start,target) <= joint.raw_max-16:
            raise ValueError('relative move too close to endpoint/wrap; do not guess branch')
        return {'joint':name, 'start':start, 'target':target, 'delta':delta,
                'duration_min_s':abs(delta)/40, 'writes':False}

    def arm(self, supported=False, commissioning=False):
        with self.lock:
            if not supported or self.state != 'disarmed':
                raise ValueError('supported, disarmed state required')
            if not commissioning: self.profile.require_execution_validated()
            first = self._snapshot(torque=0)
            self.sleep(.05)
            second = self._snapshot(torque=0)
            if any(abs(first['joints'][n]['position']-second['joints'][n]['position'])>1 for n in self.names):
                raise ValueError('arm is moving; prime refused')
            enabled = []
            goal_attempted = False
            try:
                # A lost packet or implicit torque change requires fresh fault
                # feedback even before explicit torque writes. Native hardware
                # receives all neutral goals together, avoiding stale goals on
                # other joints if firmware activates during priming.
                goal_attempted = True
                if callable(getattr(self.bus,'prime_goals',None)):
                    self.bus.prime_goals({n:second['joints'][n]['position'] for n in self.names})
                else:
                    for name in self.names:
                        self.bus.goal(name,second['joints'][name]['position'])
                primed = self._snapshot(torque=0)
                for name in self.names:
                    row = primed['joints'][name]
                    if (row['goal'] != second['joints'][name]['position']
                            or abs(row['position']-second['joints'][name]['position'])>1):
                        raise ValueError('neutral goal readback/current position changed')
                for name in self.names:
                    enabled.append(name)  # A failing ACK can still mean torque actually enabled.
                    self.bus.torque(name,True)
                feedback = self._snapshot(torque=1)
                if any(abs(feedback['joints'][n]['position']-second['joints'][n]['position'])>4 for n in self.names):
                    raise ValueError('unexpected movement while enabling torque')
                self.state, self.commissioning = 'armed', bool(commissioning)
                self._reset_hold()
                self.last_fault = None
            except BaseException as exc:
                self._fault(exc,try_hold=goal_attempted or bool(enabled))
                raise

    def _fault(self, exc, try_hold=True):
        self.state, self.last_fault, self.hold_confirmed = 'fault',str(exc),False
        if try_hold:
            self._reset_hold()
            try:
                self._hold_snapshot(self._snapshot(fault_on_error=False))
            except BaseException as hold_exc:
                self.hold_error = str(hold_exc)

    def _reset_hold(self):
        self.hold_confirmed, self.hold_error = False, None
        self.hold_targets = {}
        self.hold_offsets = {}
        self.hold_results = {n:{'goal_written':False,'goal_confirmed':False,
            'confirmed':False,'torque':None,'position_confirmed':False,'error':None}
            for n in self.names}

    def _hold_snapshot(self, data, offsets=None):
        """현재 위치(targets)를 붙잡는다. offsets(관절별 펄스)가 있으면 그 관절은 목표를 위치보다 offset만큼 덜 써서
        (goal = target - offset), 하중 때문에 목표에서 일정하게 밀려 멈추는 관절이 실제로 target 위치에 서게 한다.
        확인 기준은 그대로다: 위치가 target에서 2펄스 안, 1펄스 이하 움직임이 3번 연속, 읽은 목표가 쓴 목표와 같음."""
        self._reset_hold()
        targets = {n:data['joints'][n]['position'] for n in self.names}
        offsets = offsets or {}
        goals = {n:targets[n]-int(offsets.get(n,0)) for n in self.names}
        self.hold_targets = dict(targets)
        self.hold_offsets = {n:int(offsets.get(n,0)) for n in self.names}
        for name in self.names:
            status = self.hold_results[name]
            status['torque'] = data['joints'][name]['torque']
            if status['torque'] != 1:
                status['error'] = 'torque is not enabled; physical hold unconfirmed'
                continue
            try:
                self.bus.goal(name,goals[name])
                status['goal_written'] = True
            except Exception as exc:
                # An ACK failure on one axis must not prevent other axes' hold.
                status['error'] = str(exc)
        eligible = [n for n in self.names if self.hold_results[n]['goal_written']]
        stable = dict.fromkeys(self.names,0)
        previous = targets
        trace = {n:[] for n in self.names}      # 진단용: 읽을 때마다 (위치, 목표). 실패하면 오류 메시지에 마지막 값들을 붙인다.
        for _ in range(40):
            try: result = self._snapshot(fault_on_error=False)
            except BaseException:
                for status in self.hold_results.values():
                    status['confirmed'] = status['position_confirmed'] = False
                raise
            for name in self.names:
                row, status = result['joints'][name], self.hold_results[name]
                trace[name].append((row['position'],row['goal']))
                status['torque'] = row['torque']
                status['goal_confirmed'] = row['goal'] == goals[name]
                settled = (row['torque'] == 1 and status['goal_confirmed']
                    and abs(row['position']-targets[name]) <= guard_mode.pulses(2)
                    and abs(row['position']-previous[name]) <= 1)
                stable[name] = stable[name]+1 if settled else 0
                status['position_confirmed'] = stable[name] >= 3
                status['confirmed'] = status['goal_written'] and status['position_confirmed']
            previous = {n:result['joints'][n]['position'] for n in self.names}
            if all(self.hold_results[n]['confirmed'] for n in eligible): break
            self.sleep(.05)
        self.hold_confirmed = all(s['confirmed'] for s in self.hold_results.values())
        if not self.hold_confirmed:
            failed = [n for n,s in self.hold_results.items() if not s['confirmed']]
            detail = '; '.join(f"{n}: target {targets[n]} reads(pos/goal) "+' '.join(f'{p}/{g}' for p,g in trace[n][-10:]) for n in failed)
            self.hold_trace = detail
            self._last_hold_trace = {n:list(trace[n]) for n in failed}
            raise ValueError('physical hold unconfirmed: '+', '.join(failed)+' | '+detail)
        return result

    def hold_offset(self, name):
        """확인된 hold가 목표를 위치보다 덜 쓴 펄스 수(위치-목표). 확인된 hold가 없으면 0."""
        return self.hold_offsets.get(name,0) if self.hold_confirmed else 0

    def hold(self, attempts=1, settle_s=0.):
        """현재 자세를 붙잡고 실제로 멈춰 있는지 확인한다. attempts>1이면, 토크가 켜져 있고 쓰기 오류가 없는데 아직 자리를 못 잡았을 때만
        (게인을 바꾼 직후 하중 받는 관절이 새 평형으로 1~2펄스 움직이는 중) settle_s 기다린 뒤 현재 자세로 처음부터 다시 잡는다.
        성공 기준은 매번 같다(3번 연속 목표 2펄스 안·1펄스 이하 움직임). 토크 꺼짐·쓰기 오류·읽기 오류는 재시도하지 않고 바로 실패한다."""
        with self.lock:
            if self.state == 'disarmed': raise ValueError('hold requires armed/fault state')
            try: return self._compensated_hold(attempts,settle_s)
            except BaseException as exc:
                self._fault(exc,try_hold=False)
                raise

    def _compensated_hold(self, attempts, settle_s, first=None):
        """hold()의 재시도·하중 보상 루프. 실패하면 마지막 예외를 그대로 올린다(상태 전환은 호출자 몫).
        first가 있으면 첫 시도는 그 스냅샷을 쓴다(인계 때 방금 읽은 값)."""
        offsets, seen = {}, {}
        for attempt in range(max(1, int(attempts))):
            self._reset_hold()
            try:
                snapshot = first if first is not None and attempt == 0 else self._snapshot(fault_on_error=False)
                return self._hold_snapshot(snapshot, offsets)
            except BaseException as exc:
                self.hold_error = str(exc)
                unsettled = (isinstance(exc, ValueError) and str(exc).startswith('physical hold unconfirmed')
                             and all(r['torque'] == 1 and r['error'] is None and r['goal_written'] for r in self.hold_results.values()))
                if unsettled and attempt + 1 < attempts:
                    # 하중 때문에 목표에서 일정하게 밀려 멈춘 관절(마지막 3번 읽기에서 위치·목표가 그대로, 차이 3~6펄스):
                    # 다음 시도에서 그만큼 목표를 미리 덜 써서 실제 위치가 붙잡은 자리에 서게 한다(2026-10-16 실물: 어깨 3123/3120 고정).
                    # 같은 크기로 두 번 연속 관찰된 경우만 보정한다(한 번은 그냥 다시 잡아 본다: 움직이던 관절은 대개 그것으로 선다).
                    current = {}
                    for name, reads in getattr(self, '_last_hold_trace', {}).items():
                        tail = reads[-10:]                      # 약 0.5초 동안 위치·목표가 전혀 안 변한 경우만(움직이는 중이면 보정하지 않는다)
                        if len(tail) == 10 and len(set(tail)) == 1:
                            offset = tail[0][0] - tail[0][1]
                            if 2 < abs(offset) <= self.max_load_offset:
                                current[name] = offset
                                if seen.get(name) == offset:
                                    offsets[name] = offsets.get(name, 0) + offset
                    seen = current
                    self.sleep(settle_s)
                    continue
                raise

    def hold_existing_torque(self, supported=False, *, keep_goal=False, load_offset=False, attempts=1, settle_s=0., takeover_attempts=1):
        """Explicit supported takeover for hold only; never toggle torque or arm IK."""
        with self.lock:
            if type(keep_goal) is not bool or type(load_offset) is not bool:
                raise ValueError('keep_goal/load_offset must be boolean')
            if load_offset and not keep_goal:
                raise ValueError('load_offset only applies to adopting a kept goal')
            if not supported or self.state != 'disarmed':
                raise ValueError('supported, disarmed process required for existing torque hold')
            # 7Hz 떨림으로 읽기 쌍 사이 위치가 1~2펄스 달라질 수 있다(2026-10-19 실물). takeover_attempts>1이면
            # 위치만 3펄스 이내로 흔들린 경우에 한해 정지한 쌍이 나올 때까지 다시 읽는다. 목표가 바뀌었거나 3펄스를 넘게
            # 움직였거나 끝까지 정지한 쌍이 없으면 같은 오류로 거부한다. 1펄스 기준은 그대로다.
            for attempt in range(max(1, int(takeover_attempts))):
                first = self._snapshot(torque=1)
                self.sleep(.05)
                second = self._snapshot(torque=1)
                moves = [abs(second['joints'][n]['position']-first['joints'][n]['position']) for n in self.names]
                goal_changed = any(second['joints'][n]['goal'] != first['joints'][n]['goal'] for n in self.names)
                if max(moves) <= 1 and not goal_changed: break
                if goal_changed or max(moves) > guard_mode.pulses(3) or attempt + 1 >= takeover_attempts:
                    raise ValueError('position/goal changed; existing torque takeover refused')
                self.sleep(settle_s)
            for joint in self.profile.joints:
                name = joint.name
                row = second['joints'][name]
                if (type(row['goal']) is not int or abs(row['goal']-row['position'])>self.max_pending_pulses
                        or not joint.raw_min+16 <= row['position'] <= joint.raw_max-16):
                    raise ValueError('unsafe pending goal or endpoint; existing torque takeover refused')
            if keep_goal:
                # Repriming a loaded joint can move its equilibrium and create
                # a new sag. Adopt only already settled, unchanged goals; no
                # register writes. Strictly the goal must equal the position
                # within 2 pulses. With load_offset the goal may instead sit a
                # steady 3-6 pulses off (the compensated hold written earlier by
                # hold()/attempts>1): the position is still 2 pulses of a constant
                # offset, and that offset is recorded so later checks use it.
                self._reset_hold()
                goals = {n:first['joints'][n]['goal'] for n in self.names}
                self.sleep(.05)
                samples = [first, second, self._snapshot(torque=1)]
                previous = samples[0]
                offsets = {n:first['joints'][n]['position']-goals[n] for n in self.names}
                if load_offset:
                    offsets = {n:(o if 2 < abs(o) <= self.max_load_offset else 0) for n,o in offsets.items()}
                else:
                    offsets = dict.fromkeys(self.names,0)
                for sample in samples:
                    for joint in self.profile.joints:
                        n = joint.name
                        row = sample['joints'][n]
                        if (row['goal'] != goals[n] or abs(row['position']-goals[n]-offsets[n]) > guard_mode.pulses(2)
                                or abs(row['position']-previous['joints'][n]['position']) > 1
                                or not joint.raw_min+16 <= row['position'] <= joint.raw_max-16
                                or not joint.raw_min+16 <= goals[n] <= joint.raw_max-16):
                            raise ValueError('existing fixed goal is not stationary; takeover refused')
                    previous = sample
                last = samples[-1]
                self.hold_targets = {n:last['joints'][n]['position'] if offsets[n] else goals[n] for n in self.names}
                self.hold_offsets = dict(offsets)
                for n in self.names:
                    self.hold_results[n].update(goal_written=False,goal_confirmed=True,
                                               confirmed=True,torque=1,position_confirmed=True)
                self.hold_confirmed = True
                self.state, self.commissioning = 'hold_only', True
                return last
            self.state, self.commissioning = 'hold_only', True
            try:
                # attempts>1: same bounded steady-load compensation as hold().
                if attempts > 1: return self._compensated_hold(attempts,settle_s,first=second)
                return self._hold_snapshot(second)
            except BaseException as exc:
                self.hold_error = str(exc)
                self._fault(exc,try_hold=False)
                raise

    def observe_hold(self, duration):
        """Verify the original fixed goals throughout the trial; never recenter success."""
        duration = finite(duration,'hold duration')
        if not 0 < duration <= 30: raise ValueError('hold duration must be >0 and <=30s')
        with self.lock:
            if self.state not in ('armed','hold_only') or not self.hold_confirmed:
                raise ValueError('confirmed hold required for observation')
            targets = dict(self.hold_targets)
            if set(targets) != set(self.names):raise ValueError('confirmed hold targets missing')
            samples, spans = 0, {n:[] for n in self.names}
            start = self.clock()
            try:
                while True:
                    data = self._snapshot(torque=1)
                    for name in self.names:
                        row = data['joints'][name]
                        if (row['goal'] != targets[name]-self.hold_offsets.get(name,0)
                                or abs(row['position']-targets[name])>guard_mode.pulses(2)):
                            raise ValueError(f'hold trial failed: {name} fixed goal changed or tracking error')
                        spans[name].append(row['position'])
                    samples += 1
                    if self.clock()-start >= duration: break
                    self.sleep(.05)
                data['hold_trial_succeeded'] = True
                data['observation'] = {'duration_s':self.clock()-start,'samples':samples,
                    'fixed_goals':targets,'position_span_pulse':{n:max(v)-min(v) for n,v in spans.items()},
                    'max_goal_error_pulse':{n:max(abs(x-targets[n]) for x in v) for n,v in spans.items()}}
                return data
            except BaseException as exc:
                if self.state != 'fault': self._fault(exc)
                raise

    def probe_base(self, delta=4, supported=False):
        """Supported raw-pulse base-only round trip; never grants general arming."""
        with self.lock:
            self.last_probe = None
            if (not supported or self.state != 'hold_only' or not self.hold_confirmed
                    or type(delta) is not int or not 1 <= abs(delta) <= 8):
                raise ValueError('base probe requires supported confirmed hold and 1-8 pulse delta')
            baseline = self._snapshot(torque=1)
            expected = dict(self.hold_targets)
            if set(expected) != set(self.names):raise ValueError('confirmed hold targets missing')
            if any(baseline['joints'][n]['goal'] != expected[n]
                    or abs(baseline['joints'][n]['position']-expected[n])>2 for n in self.names):
                exc = ValueError('base probe initial hold changed or tracking error')
                self._fault(exc)
                raise exc
            reference = copy.deepcopy(baseline)
            reference['joints']['base']['position'] = expected['base']
            target = self._relative_target(reference,'base',delta)
            endpoint_tolerance = 2  # Same raw-position contract as hold/relative/execute.
            base_mapping = self.profile.joints[0]
            base_mapping.raw_to_rad(target['start'])
            base_mapping.raw_to_rad(target['target'])
            previous = target['start']
            samples = []
            self.last_probe = {'succeeded':False, 'origin_raw':target['start'],
                'target_raw':target['target'], 'delta_pulse':delta,
                'endpoint_tolerance_pulse':endpoint_tolerance,
                'phase':'outward', 'samples':samples, 'error':None}

            def check():
                data = self._snapshot(torque=1)
                samples.append(copy.deepcopy(data))
                for name in self.names:
                    row = data['joints'][name]
                    # Let a bounded probe reach its requested excitation before
                    # declaring startup lag. The absolute excursion guard below
                    # and strict endpoint/return checks still apply.
                    limit = max(4,abs(delta)) if name=='base' else 2
                    if row['goal'] != expected[name] or abs(row['position']-expected[name])>limit:
                        raise ValueError(f'base probe tracking/goal error: {name}')
                if abs(data['joints']['base']['position']-target['start'])>abs(delta)+2:
                    raise ValueError('base probe exceeded permitted excursion')
                return data

            def move(destination):
                nonlocal previous
                direction = 1 if destination>previous else -1
                for raw in range(previous+direction,destination+direction,direction):
                    check()
                    self.bus.goal('base',raw)
                    expected['base'] = raw
                    previous = raw
                    self.sleep(.025)
                stable = 0
                last_position = None
                for _ in range(40):
                    data = check()
                    position = data['joints']['base']['position']
                    error = abs(position-destination)
                    settled = last_position is None or abs(position-last_position)<=1
                    stable = stable+1 if error<=endpoint_tolerance and settled else 0
                    last_position = position
                    if stable>=3:return data
                    self.sleep(.05)
                raise ValueError('base probe endpoint not reached')

            try:
                check()
                outward = move(target['target'])
                self.last_probe['phase'] = 'return'
                returned = move(target['start'])
                self.hold_targets = dict(expected)
                observation = self.observe_hold(.5)
                self.last_probe.update(succeeded=True, phase='complete')
                return {'base_probe_succeeded':True,'delta_pulse':delta,
                    'endpoint_tolerance_pulse':endpoint_tolerance,
                    'outward_observed_delta_pulse':outward['joints']['base']['position']-target['start'],
                    'outward_goal_error_pulse':abs(outward['joints']['base']['position']-target['target']),
                    'return_goal_error_pulse':abs(returned['joints']['base']['position']-target['start']),
                    'commanded_delta_deg':delta*360/4096,'origin_raw':target['start'],
                    'outward':outward,'returned':returned,'observation':observation['observation'],
                    'samples_raw':samples,'torque_writes':False,
                    'physical_geometric_mapping_validated':False}
            except BaseException as exc:
                self.last_probe['error'] = str(exc)
                if self.state != 'fault':self._fault(exc)
                raise

    def disarm(self, supported=False):
        with self.lock:
            if not supported: raise ValueError('support arm before torque off')
            try:
                self._snapshot()
                for name in self.names: self.bus.torque(name,False)
                result = self._snapshot(torque=0)
                self.state = 'disarmed'
                self._reset_hold()
                return result
            except BaseException as exc:
                self._fault(exc,try_hold=False)
                raise

    def relative(self, name, delta, supported=False):
        with self.lock:
            if self.state != 'armed' or not supported:
                raise ValueError('armed and supported required for relative trial')
            initial = self._snapshot(torque=1)
            target = self._relative_target(initial,name,delta)
            direction = 1 if delta > 0 else -1
            self._reset_hold()
            try:
                for offset in range(1,abs(delta)+1):
                    feedback = self._snapshot(torque=1)
                    previous = target['start']+(offset-1)*direction
                    if abs(feedback['joints'][name]['position']-previous)>8:
                        raise ValueError('relative tracking error')
                    if any(abs(feedback['joints'][n]['position']-initial['joints'][n]['position'])>4 for n in self.names if n!=name):
                        raise ValueError('uncommanded joint movement')
                    self.bus.goal(name,target['start']+offset*direction)
                    self.sleep(1/40)
                for _ in range(40):
                    feedback = self._snapshot(torque=1)
                    if abs(feedback['joints'][name]['position']-target['target'])<=2:
                        return feedback
                    self.sleep(.05)
                raise ValueError('relative goal not reached')
            except BaseException as exc:
                if self.state != 'fault': self._fault(exc)
                raise

    def prepare_trajectory(self, names, points):
        """Serialize ROS admission with monitoring, arm/disarm and execution."""
        with self.lock:
            if self.state != 'armed' or self.commissioning:
                raise ValueError('validated normal arm required for trajectory')
            try: self.profile.require_execution_validated()
            except BaseException as exc:
                self._fault(exc)
                raise
            from hardware.servo_trajectory import validate_trajectory
            initial = self._snapshot(torque=1)
            current = [j.raw_to_rad(initial['joints'][j.name]['position']) for j in self.profile.joints]
            return validate_trajectory(names,points,current,self.profile.joints)

    def execute(self, plan, cancelled=lambda:False):
        with self.lock:
            plan = self.prepare_trajectory(list(plan.names),plan.points)
            initial = self._snapshot(torque=1)
            last = {n:initial['joints'][n]['position'] for n in self.names}
            direction = dict.fromkeys(self.names,0)
            period, start = .025, self.clock()
            tick = 0
            self._reset_hold()
            try:
                while True:
                    if cancelled():
                        self.hold()
                        return 'cancelled'
                    feedback = self._snapshot(torque=1)
                    if any(abs(feedback['joints'][n]['position']-last[n])>16 for n in self.names):
                        raise ValueError('trajectory tracking error')
                    elapsed = self.clock()-start
                    ending = elapsed >= plan.duration
                    values = plan.sample(elapsed)
                    raw = {j.name:j.rad_to_raw(value) for j,value in zip(self.profile.joints,values)}
                    if any(abs(raw[n]-last[n])>8 for n in self.names):
                        raise ValueError('stale loop/too large interpolated step')
                    if not ending:  # the exact final target is always commanded
                        raw = {n:reversal_filtered(raw[n],last[n],direction[n]) for n in self.names}
                    for name in self.names:
                        if raw[name] != last[name]:
                            self.bus.goal(name,raw[name])
                            direction[name] = 1 if raw[name] > last[name] else -1
                    last = raw
                    if ending: break
                    # Fixed-rate schedule: read/write time must not stretch the
                    # command period (uneven spacing makes uneven servo steps).
                    tick += 1
                    self.sleep(max(0., start+tick*period-self.clock()))
                for _ in range(40):
                    if cancelled(): self.hold(); return 'cancelled'
                    final = self._snapshot(torque=1)
                    if all(abs(final['joints'][n]['position']-last[n])<=2 for n in self.names):
                        return 'succeeded'
                    self.sleep(.05)
                raise ValueError('trajectory target not reached')
            except BaseException as exc:
                if self.state != 'fault': self._fault(exc)
                raise

    def close(self, timeout=3):
        """Close only after all I/O owners finish; retain torque and fault evidence."""
        if not self.lock.acquire(timeout=timeout):
            raise TimeoutError('driver I/O still active; transport remains open')
        try:
            if self.closed: return
            if self.state in ('armed','hold_only','fault'):
                try: self.hold()
                except BaseException as exc: self.shutdown_error = str(exc)
            self.bus.close()
            self.closed = True
        finally: self.lock.release()
