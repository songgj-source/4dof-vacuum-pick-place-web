"""든 채 이동(place) 실행: place_plan.plan_place의 계획을 작은 이동 단계로 쪼개 순서대로 실행한다.

이 모듈은 장치를 직접 열지 않는다. 이동(hover), 현재 자세(fresh_xyz), 해제(release), 취소(cancelled)는
호출자(pick_auto.py)가 넘겨 주는 함수이고, 그래서 펌프를 쥔 프로세스 안에서 실행된다.

hover_trial.run_hover_trial의 제약에 맞춘다:
  - 수직 올리기(lift_m): 한 번에 5~30mm → 측정 높이에서 남은 거리를 26mm 이하로 나눈다
  - 수직 내리기(measured_gap_m): 한 번에 50mm 이하 → 40mm 이하로 나눈다
  - 수평 이동: box_top_m=.040을 주어 높이 70mm 이상, 긴 구간은 30mm 단위로 쪼갠다(관절 보간이 직선에서
    벗어나는 정도를 줄이기 위해)
place 중에는 공구를 앞으로 기울여도 된다(사용자 허용, 최대 60도). 기울이는 단계(tilt/untilt)는 같은 자리에서
자세만 바꾸고, 기울어진 단계의 이동에는 pitch_rad를 hover에 넘긴다.
속도: 수평 이동과 70mm 이상 수직 올리기는 fast=True(호출자가 place 속도, 기본 300펄스/초로 처리),
바닥에서 처음 들기·칸 안 내리기·기울기 변경은 기존 느린 모드다(hover_trial이 접촉·들기 구간의 빠른 모드를 금지).
압력 센서가 없어 물체가 실제로 붙어 있는지는 확인하지 못한다. 이동 중 실패하거나 시간 제한을 넘기면
되돌리거나 내려놓지 않고 멈추며(해제하지 않음), 이후 프로세스 종료 시 펌프가 정지되어 물체가 떨어질 수 있다.
"""
import math
import time

from motion import guard_mode
from motion.place_exit import ExitPathRefused
from sorting.carry_path import polar_path

LIFT_MAX_M = .036
LIFT_MIN_M = .005
LOWER_MAX_M = .045
LIFT_CHUNK_M = .0358           # 한 번에 올리는 양(hover_trial 한도 36mm 바로 아래: 흰 상자는 lift20 뒤 약 34.6mm를 한 번에 올라야 한다)
LOWER_CHUNK_M = .049           # 한 번에 내리는 양(hover_trial 한도 50mm): 칸 안 내리기를 한 구간으로 합친다(사용자: 속도 우선, 2026-10-19)
VERTICAL_DONE_M = .004
FAST_MIN_Z = .070              # 이 높이 이상에서의 수직 올리기·수평 이동만 빠른 프로파일(place 속도)
FAST_START_Z = .068
MAX_VERTICAL_CHUNKS = 12
HORIZONTAL_STEP_M = .030
MIN_HORIZONTAL_Z = .070
BOX_TOP_M = .040
START_TOLERANCE_M = .005
STEP_XY_TOLERANCE_M = .003
RELEASE_XY_TOLERANCE_M = .005
RELEASE_Z_TOLERANCE_M = .004
PRE_LOWER_XY_TOLERANCE_M = .002
MAX_PRE_LOWER_ERROR_M = .010
MAX_PRE_LOWER_ALIGNMENT_MOVES = 2
DEFAULT_MAX_CARRY_S = 180.
VERTICAL_LEGS = {'raise_clear': 'up', 'raise_low': 'up', 'raise': 'up', 'retreat': 'up', 'lower': 'down'}
HORIZONTAL_LEGS = ('pull_in', 'carry', 'retire')
PITCH_LEGS = ('tilt', 'untilt')          # 같은 자리에서 공구 기울기만 바꾼다
VERTICAL_PITCH = -math.pi / 2
MAX_TILT_RAD = math.radians(60.)          # 2026-10-21 사용자: 50→60(트레이 1번 칸 36.7cm. 흰 상자는 50도로 36.6cm까지, 60도로 37.6cm까지)
MAX_TILT_STEP_RAD = math.radians(15.)      # 제자리 기울이기는 15도 이하로 나눈다: 한 번에 45도면 관절 보간 중 컵 끝이 약 5mm 가라앉는다(실물)
HIGH_TILT_Z = .100                         # 2026-10-20: 이 높이 이상에서는(상자 위 안전선 65mm보다 35mm 이상 위) 5mm 가라앉아도 여유가 커서
HIGH_TILT_STEP_RAD = math.radians(50.)     # 제자리 기울이기를 한 번에 한다(1번 칸 50도: 15도씩 4번 멈추던 것을 1번으로)


def _split(amount, limit, label):
    """부호 있는 이동량을 limit 이하의 같은 크기 단계 여러 개로 나눈다. 5mm 미만 단계는 만들 수 없다."""
    size = abs(amount)
    if size < .002:
        return []
    count = math.ceil(size / limit - 1e-9)
    step = size / count
    if step < LIFT_MIN_M - 1e-9:
        raise ValueError(f'{label}: a {size * 1000:.1f} mm vertical move is below the 5 mm minimum step')
    return [math.copysign(step, amount)] * count


def _leg_pitch(leg):
    """leg의 pitch_rad(없으면 None=기본 수직). 수직보다 뒤로 기울거나 30도를 넘으면 거부한다."""
    pitch = leg.get('pitch_rad')
    if pitch is None:
        if leg['name'] in PITCH_LEGS:
            raise ValueError(f'leg {leg["name"]} needs pitch_rad')
        return None
    if isinstance(pitch, bool) or not isinstance(pitch, (int, float)) or not math.isfinite(pitch) \
            or not VERTICAL_PITCH - 1e-9 <= pitch <= VERTICAL_PITCH + MAX_TILT_RAD + 1e-9:
        raise ValueError(f'leg {leg["name"]}: pitch must be straight down or tilted forward by at most 60 degrees')
    return float(pitch)


def build_steps(plan, start):
    """계획 → 실행 단계 목록(이동 전에 모두 검사). 단계는 dict(name, leg, kind, target[, lift|gap])."""
    if plan.get('executed') is not False or not plan.get('legs'):
        raise ValueError('plan must be an unexecuted plan with legs')
    cur = tuple(float(v) for v in start)
    steps = []
    for leg in plan['legs']:
        name, target = leg['name'], tuple(float(v) for v in leg['target'])
        _leg_pitch(leg)
        if name in PITCH_LEGS:
            if math.dist(target, cur) > .001 or target[2] < MIN_HORIZONTAL_Z - 1e-9:
                raise ValueError(f'leg {name} must change the tool pitch in place at {MIN_HORIZONTAL_Z * 1000:.0f} mm or higher')
            steps.append(dict(name=f'place_{name}_0', leg=name, kind='pitch', target=target))
        elif name in VERTICAL_LEGS:
            if math.hypot(target[0] - cur[0], target[1] - cur[1]) > .001:
                raise ValueError(f'leg {name} must be vertical')
            parts = _split(target[2] - cur[2], LIFT_MAX_M if VERTICAL_LEGS[name] == 'up' else LOWER_MAX_M, name)
            z = cur[2]
            for i, delta in enumerate(parts):
                z += delta
                point = (cur[0], cur[1], z)
                steps.append(dict(name=f'place_{name}_{i}', leg=name, kind=VERTICAL_LEGS[name], target=point, delta=delta))
            cur = (cur[0], cur[1], cur[2] + sum(parts)) if parts else (cur[0], cur[1], target[2])    # 2mm 미만 차이는 도달한 것으로 본다
        elif name in HORIZONTAL_LEGS:
            if target[2] < MIN_HORIZONTAL_Z - 1e-9 or abs(target[2] - cur[2]) > .001:
                raise ValueError(f'leg {name} must be horizontal at {MIN_HORIZONTAL_Z * 1000:.0f} mm or higher')
            distance = math.hypot(target[0] - cur[0], target[1] - cur[1])
            count = max(1, math.ceil(distance / HORIZONTAL_STEP_M - 1e-9))
            for i in range(1, count + 1):
                f = i / count
                point = (cur[0] + (target[0] - cur[0]) * f, cur[1] + (target[1] - cur[1]) * f, target[2])
                steps.append(dict(name=f'place_{name}_{i - 1}', leg=name, kind='horizontal', target=point))
            cur = target
        else:
            raise ValueError(f'unknown place leg {name!r}')
    return steps


OUTWARD_CLAMP_MAX_M = .015


def _not_outward(cur, target, name):
    """제자리 기울기 변경의 xy: 측정 위치를 쓰되, 계획 목표의 반경보다 바깥이면 그 반경까지만 안쪽으로 보정한다.
    수직 도달 한계(z=0.10m에서 약 0.25m) 경계에서는 서보가 1~3mm만 바깥에 서도 역기구학 해가 없어진다(2026-10-19: 계획 0.2492m,
    실제 0.2505m → 'hover has no direct-branch solution', 상자는 이미 놓은 뒤였다). 계획은 그 반경에서 해가 있음을 검증했다.
    계획과 너무 멀리 떨어져 있으면(OUTWARD_CLAMP_MAX_M 초과) 움직이지 않고 거부한다."""
    r_cur, r_target = math.hypot(cur[0], cur[1]), math.hypot(target[0], target[1])
    if r_cur <= r_target + 1e-9 or r_cur < 1e-9:
        return cur[0], cur[1]
    if r_cur - r_target > guard_mode.length(OUTWARD_CLAMP_MAX_M):
        raise ValueError(f'{name}: arm is {1000 * (r_cur - r_target):.0f} mm outside the planned radius; refusing to change the tool pitch there')
    k = r_target / r_cur
    return cur[0] * k, cur[1] * k


EXIT_RISE_STEP_M = .010         # 한 번에 빠져나오기: 칸 안 수직 상승 점 간격
EXIT_SLOW_BELOW_Z = .070        # 이 높이 아래(칸 안)는 느린 2펄스/틱, 위는 4펄스/틱(예전 단계별 이동과 같은 속도)


def build_exit_transition(legs, start, start_pitch, *, include_untilt=True):
    """놓은 뒤 남은 계획이 retreat[, retire][, untilt] 순서일 때, 멈추지 않고 한 번에 갈 경로 점을 만든다(이동 없음).
    retreat: 놓은 자리에서 넣을 때의 기울기 그대로 수직 상승(10mm 간격). retire: 계획 경유점(30mm 간격)과 경유점별 손목 각도
    (place_plan이 도달을 검사한 retire_pitches). untilt: 마지막 자리에서 15도씩 수직으로. 그 밖의 계획이면 None(예전처럼 단계별)."""
    names = [leg['name'] for leg in legs]
    if not names or names[0] != 'retreat' or names not in (['retreat'], ['retreat', 'retire'], ['retreat', 'untilt'],
                                                           ['retreat', 'retire', 'untilt']):
        return None
    start = tuple(float(v) for v in start)
    clear_z = float(legs[0]['target'][2])
    if not start[2] + .005 < clear_z <= .150:
        return None
    pitch = float(start_pitch)
    points = []
    count = math.ceil((clear_z - start[2]) / EXIT_RISE_STEP_M - 1e-9)
    for i in range(1, count + 1):
        z = clear_z if i == count else start[2] + (clear_z - start[2]) * i / count
        points.append(dict(xyz=(start[0], start[1], z), pitch_rad=pitch, pulses=2 if z < EXIT_SLOW_BELOW_Z else 4))
    here = (start[0], start[1], clear_z)
    for leg in legs[1:]:
        target = tuple(float(v) for v in leg['target'])
        if leg['name'] == 'retire':
            if abs(target[2] - clear_z) > 1e-6:
                return None
            route = polar_path(here, target, HORIZONTAL_STEP_M)
            plan_pitches = [float(v) for v in leg.get('retire_pitches') or []]
            if len(plan_pitches) > len(route):
                return None
            pitches = [pitch] * (len(route) - len(plan_pitches)) + plan_pitches if plan_pitches else [_leg_pitch(leg) or pitch] * len(route)
            for point, step_pitch in zip(route, pitches):
                if abs(step_pitch - pitch) > MAX_TILT_STEP_RAD + 1e-3 or not VERTICAL_PITCH - 1e-3 <= step_pitch <= pitch + 1e-3:
                    return None                                  # 세우기만, 15도 이하씩(예전 retire와 같은 조건)
                points.append(dict(xyz=tuple(point), pitch_rad=step_pitch, pulses=4))
                pitch = step_pitch
            here = target
        elif leg['name'] == 'untilt':
            if leg is legs[-1] and not include_untilt:
                break                                            # 곧바로 HOME 복귀라 세우지 않는다(예전과 같다)
            goal = _leg_pitch(leg)
            count = math.ceil(abs(goal - pitch) / MAX_TILT_STEP_RAD - 1e-9)
            for i in range(1, count + 1):
                points.append(dict(xyz=here, pitch_rad=goal if i == count else pitch + (goal - pitch) * i / count, pulses=4))
            pitch = goal
    return dict(start=start, start_pitch=float(start_pitch), clear_z=clear_z, target=points[-1]['xyz'], end_pitch=pitch,
                waypoints=points)


def run_place(plan, *, hover, fresh_xyz, release, cancelled, empty_run=False, skip_final_untilt=False, max_carry_s=DEFAULT_MAX_CARRY_S,
              now=time.monotonic, record=lambda event: None, exit_path=None):
    """exit_path(transition): 있으면 놓은 뒤 retreat→retire→untilt를 멈추지 않는 한 번의 경로로 실행한다(build_exit_transition).
    준비에서 거부되면(ExitPathRefused, 움직이기 전) 예전처럼 단계별로 간다."""
    if not plan.get('start'):
        raise ValueError('plan has no start pose')
    start = tuple(fresh_xyz())
    build_steps(plan, plan['start'])                     # 이동 전에 계획 전체를 검사한다(잘못되면 여기서 ValueError)
    if math.dist(start, [float(v) for v in plan['start']]) > guard_mode.length(START_TOLERANCE_M):
        raise ValueError(f'start pose differs from the plan: measured {tuple(round(v, 4) for v in start)} '
                         f'vs planned {tuple(round(float(v), 4) for v in plan["start"])}')
    began = now()
    slot_xyz = tuple(float(v) for v in next(l for l in plan['legs'] if l['name'] == 'lower')['target'])
    released, done, merged = False, [], []

    def checkpoint(next_name):
        if cancelled():
            raise InterruptedError('place cancelled; not releasing')
        if now() - began > max_carry_s:
            raise TimeoutError(f'place exceeded {max_carry_s:.0f} s before step {next_name}; stopped without releasing')

    def waypoints(p0, p1):
        """p0에서 p1까지 30mm 이하 간격의 중간점들(p1 포함). 극좌표 경로(base 각도 일정): 경유점으로 멈추지 않고 지나간다."""
        return polar_path(p0, p1, HORIZONTAL_STEP_M)

    def move_path(step_name, points, **kw):
        """경유점을 지나는 연속 이동. hover_trial이 1회 이동량(1600펄스) 초과로 거부하면(움직이기 전에 거부) 경로를 반으로 나눠 이어 간다."""
        try:
            move(step_name, points[-1], via=points[:-1], **kw)
        except ValueError as exc:
            if str(exc) != 'hover change exceeds trial envelope' or len(points) < 2:
                raise
            mid = len(points) // 2
            move_path(step_name + 'a', points[:mid], **kw)
            move_path(step_name + 'b', points[mid:], **kw)

    def move(step_name, point, **kw):
        checkpoint(step_name)
        hover(step_name, point, **kw)
        done.append(step_name)
        record(dict(step=step_name, target=point, **{k: v for k, v in kw.items() if k in ('pitch_rad',)}))

    def move_horizontal(step_name, leg, points, extra, after):
        """수평 이동 하나를 실행한다. 놓은 뒤 후퇴(retire)에 계획이 경유점별 pitch(retire_pitches: place_plan이 모든 지점의 도달을 검사한 값)를
        주었으면, 같은 pitch가 이어지는 경유점끼리 한 이동으로 묶어 순서대로 실행한다: 후퇴하는 동안 공구를 15도 이하씩 세우고,
        남은 기울기는 뒤따르는 untilt가 제자리에서 세운다. 그렇지 않으면 한 번에 간다."""
        nonlocal current_pitch
        plan_pitches = leg.get('retire_pitches')
        if leg['name'] == 'retire' and plan_pitches and len(points) >= len(plan_pitches):
            pitches = [plan_pitches[0]] * (len(points) - len(plan_pitches)) + [float(v) for v in plan_pitches]
            previous = current_pitch
            groups = []
            for point, pitch in zip(points, pitches):
                if abs(pitch - previous) > MAX_TILT_STEP_RAD + 1e-6 or not VERTICAL_PITCH - 1e-9 <= pitch <= current_pitch + 1e-9:
                    groups = None
                    break
                if groups and abs(groups[-1][0] - pitch) < 1e-9:
                    groups[-1][1].append(point)
                else:
                    groups.append((pitch, [point]))
                previous = pitch
            if groups:
                for i, (pitch, chunk) in enumerate(groups):
                    move_path(f'{step_name[:-2]}_{i}', chunk, box_top_m=BOX_TOP_M, fast=True, pitch_rad=pitch)
                current_pitch = groups[-1][0]
                if len(groups) > 1 or abs(groups[0][0] - extra.get('pitch_rad', groups[0][0])) > 1e-9:
                    merged.append('retire+untilt')
                return after
        move_path(step_name, points, box_top_m=BOX_TOP_M, fast=True, **extra)
        return after

    legs = [dict(leg) for leg in plan['legs']]
    planned = tuple(float(v) for v in plan['start'])
    current_pitch = VERTICAL_PITCH
    index = 0
    while index < len(legs):
        leg = legs[index]
        name, target = leg['name'], tuple(float(v) for v in leg['target'])
        pitch = _leg_pitch(leg)
        extra = {} if pitch is None else {'pitch_rad': pitch}
        if pitch is not None and name not in PITCH_LEGS:
            current_pitch = pitch
        index += 1
        if name in PITCH_LEGS and name == 'untilt' and abs(pitch - current_pitch) < 1e-6:
            continue                                           # 후퇴하며 이미 다 세웠다
        if name == 'untilt' and skip_final_untilt and index == len(legs):
            merged.append('untilt skipped')                    # 곧바로 HOME 복귀(관절 공간 이동)라 자세를 세우지 않아도 된다: 약 2.6초 절약
            continue
        if name in PITCH_LEGS:
            step_limit = HIGH_TILT_STEP_RAD if target[2] >= HIGH_TILT_Z - 1e-9 else MAX_TILT_STEP_RAD
            count = max(1, math.ceil(abs(pitch - current_pitch) / step_limit - 1e-9))
            for i in range(count):
                cur = tuple(fresh_xyz())
                step_pitch = current_pitch + (pitch - current_pitch) * (i + 1) / count
                x, y = _not_outward(cur, target, name)
                point = (x, y, target[2])                       # 측정한 xy에서 제자리 자세 변경(높이는 계획 값: 70mm 이상). 계획 반경보다 바깥이면 안쪽으로만 보정
                move(f'place_{name}_{i}', point, box_top_m=BOX_TOP_M, pitch_rad=step_pitch, fast=True)   # 70mm 이상 허공에서 제자리 회전: 운반과 같은 빠른 방식(느린 1펄스 방식은 단계당 약 14초)
            current_pitch = pitch
        elif name in HORIZONTAL_LEGS:
            points = waypoints(planned, target)
            index = move_horizontal(f'place_{name}_0', leg, points, extra, index)
        else:
            up = VERTICAL_LEGS[name] == 'up'
            lower_xy = slot_xyz[:2]                        # 칸 안 내리기에 명령할 xy(정렬에서 찾은 보정이 있으면 그만큼 민 값)
            if name == 'lower':
                # Align above the tray so carry error cannot accumulate with
                # descent drift. The original release tolerances stay intact.
                for attempt in range(MAX_PRE_LOWER_ALIGNMENT_MOVES + 1):
                    cur = tuple(fresh_xyz())
                    error = math.hypot(cur[0] - slot_xyz[0], cur[1] - slot_xyz[1])
                    if error <= guard_mode.length(PRE_LOWER_XY_TOLERANCE_M):
                        break
                    if (cur[2] < MIN_HORIZONTAL_Z or error > guard_mode.length(MAX_PRE_LOWER_ERROR_M)
                            or attempt == MAX_PRE_LOWER_ALIGNMENT_MOVES):
                        raise ValueError(f'pre-lower alignment refused: XY error {error * 1000:.1f} mm at z={cur[2]:.4f}; not lowering')
                    # 늘 같은 쪽으로 어긋나 서는 오차(하중·마찰)는 같은 목표를 다시 보내도 줄지 않는다(2026-10-26 Pi: 2번 칸 위에서
                    # 두 번 다 +4.0mm). 잰 오차만큼 목표를 밀어 보낸다(접근 보정과 같은 방식). 민 양은 정렬 한도 안이다.
                    lower_xy = (lower_xy[0] + slot_xyz[0] - cur[0], lower_xy[1] + slot_xyz[1] - cur[1])
                    if math.hypot(lower_xy[0] - slot_xyz[0], lower_xy[1] - slot_xyz[1]) > guard_mode.length(MAX_PRE_LOWER_ERROR_M):
                        raise ValueError(f'pre-lower alignment refused: correction exceeds the alignment envelope at z={cur[2]:.4f}; not lowering')
                    move(f'place_align_{attempt}', (*lower_xy, cur[2]),
                         box_top_m=BOX_TOP_M, fast=False, **extra)
            i = 0
            while True:
                cur = tuple(fresh_xyz())
                remaining = target[2] - cur[2]
                if abs(remaining) < VERTICAL_DONE_M:
                    break
                step_name = f'place_{name}_{i}'
                if up and cur[2] >= FAST_START_Z and target[2] >= FAST_MIN_Z - 1e-9:
                    # 이미 70mm 위: 올라간 뒤 바로 이어지는 수평 이동(carry/pull_in)이 있으면 하나로 이어서 간다
                    nxt = legs[index] if index < len(legs) else None
                    if nxt is not None and nxt['name'] in HORIZONTAL_LEGS and abs(float(nxt['target'][2]) - target[2]) < 1e-6 \
                            and _leg_pitch(nxt) == pitch:
                        end = tuple(float(v) for v in nxt['target'])
                        via = [(cur[0], cur[1], target[2]), *waypoints((cur[0], cur[1], target[2]), end)]
                        index = move_horizontal(f'place_{nxt["name"]}_0', nxt, via, extra, index + 1)
                        planned = end
                        merged.append(f'{name}+{nxt["name"]}')
                        target = (cur[0], cur[1], target[2])          # 올리기 단계는 끝난 것으로 본다
                        break
                    following = legs[index] if index < len(legs) else None
                    keep = {**extra, 'pitch_rad': current_pitch} if following is not None and following['name'] in PITCH_LEGS else extra
                    move(step_name, (cur[0], cur[1], target[2]), box_top_m=BOX_TOP_M, fast=True, **keep)     # 앞 조각이 이미 돌린 기울기를 유지한다
                else:
                    # 바닥 근처 올리기와 칸 안 내리기는 느린 lift/gap 모드. 올릴 때는 70mm까지만 느리게 간다.
                    goal_z = min(target[2], FAST_MIN_Z) if (up and target[2] >= FAST_MIN_Z) else target[2]
                    remaining = goal_z - cur[2]
                    # 측정한 현재 높이에서 남은 거리를 나눈다: 자세 오차가 쌓여도 hover_trial의 한 번 이동 한도를 넘지 않게 한다.
                    count = math.ceil(abs(remaining) / (LIFT_CHUNK_M if up else LOWER_CHUNK_M) - 1e-9)
                    chunk = remaining / count
                    if abs(chunk) < LIFT_MIN_M:
                        chunk = math.copysign(LIFT_MIN_M, remaining)
                    if name == 'lower':
                        if math.hypot(cur[0] - slot_xyz[0], cur[1] - slot_xyz[1]) > guard_mode.length(STEP_XY_TOLERANCE_M):
                            raise ValueError('drop XY drift exceeds the vertical descent envelope; not lowering')
                        point = (*lower_xy, cur[2] + chunk)        # 정렬 보정량을 유지해 내려가며 다시 밀리지 않게 한다
                    else:
                        point = (cur[0], cur[1], cur[2] + chunk)
                    if up:
                        # 올리는 동안 다음 기울이기 단계의 각도를 15도씩 같이 돌린다(별도 제자리 단계 시간을 줄인다).
                        nxt_leg = legs[index] if index < len(legs) else None
                        tilt_goal = _leg_pitch(nxt_leg) if nxt_leg is not None and nxt_leg['name'] in PITCH_LEGS else None
                        if tilt_goal is not None and abs(tilt_goal - current_pitch) > 1e-9:
                            step = math.copysign(min(MAX_TILT_STEP_RAD, abs(tilt_goal - current_pitch)), tilt_goal - current_pitch)
                            current_pitch += step
                            move(step_name, point, lift_m=chunk, **{**extra, 'pitch_rad': current_pitch})
                        else:
                            # 다음 기울이기 각도에 이미 도달했으면(올리기 여러 조각 중 뒤쪽) 그 자세를 유지한다(leg의 기울기로 되돌리지 않는다)
                            move(step_name, point, lift_m=chunk, **({**extra, 'pitch_rad': current_pitch} if tilt_goal is not None else extra))
                    else:
                        move(step_name, point, measured_gap_m=-chunk, **extra)
                i += 1
                if i > MAX_VERTICAL_CHUNKS:
                    raise ValueError(f'leg {name} did not converge to z={target[2]:.4f} after {i} steps')
            if name == 'lower':
                at = tuple(fresh_xyz())
                if (math.hypot(at[0] - slot_xyz[0], at[1] - slot_xyz[1]) > guard_mode.length(RELEASE_XY_TOLERANCE_M)
                        or abs(at[2] - slot_xyz[2]) > guard_mode.length(RELEASE_Z_TOLERANCE_M)):
                    raise ValueError(f'release refused: pose {tuple(round(v, 4) for v in at)} is not at the drop point '
                                     f'{tuple(round(v, 4) for v in slot_xyz)}')
                if not empty_run:
                    checkpoint('release')
                    release(slot_xyz)
                    released = True
                if exit_path is not None:
                    transition = build_exit_transition(legs[index:], fresh_xyz(), current_pitch,
                                                       include_untilt=not skip_final_untilt)
                    if transition is not None:
                        checkpoint('place_exit')
                        try:
                            exit_path(transition)
                        except ExitPathRefused as exc:
                            record(dict(step='place_exit_refused', reason=str(exc)))      # 움직이기 전 거부: 예전처럼 단계별로
                        else:
                            done.append('place_exit')
                            merged.append('+'.join(leg['name'] for leg in legs[index:]))
                            current_pitch = transition['end_pitch']
                            index = len(legs)
        planned = target if planned is None or name not in HORIZONTAL_LEGS or True else planned
    return dict(steps=done, merged=merged, released=released, empty_run=bool(empty_run), slot_xyz=slot_xyz, duration_s=now() - began,
                pressure_verified=False)
