"""내 주행 서버(ros2_slam test/map_web.py, 같은 Pi의 8082)의 REST API로 차를 움직인다. 쓰는 기능은 둘뿐이다.

  base_state       GET /api/state, /api/drive   위치·방향, 라벨 목록, Nav2/벽 근접 상태
  goto_label_wall  POST /api/nav/goto_wall      라벨로 이동한 뒤 정면 벽과 수직으로 맞추고 gap(m) 앞까지 전진
  goto_label_precise  POST /api/nav/goto + /api/drive   라벨의 정확한 위치·방향으로(기본). Nav2로 근처까지 간 뒤 지도 위치를 보며
                   천천히 마무리 정렬한다(라벨을 만든 그 자리, 위치 ±3 cm·방향 ±2°)

주소는 ARM_BASE_URL(기본 http://127.0.0.1:8082).
goto_wall은 접수만 하고 바로 돌아오므로, 끝날 때까지 /api/state의 nav(이동)·wall(벽 근접)을 읽어 결과를 정한다.
  nav.state   backing / sending / active / canceling → 이동 중, succeeded → 도착, failed / aborted / canceled / rejected / unavailable → 실패
  nav.then_state (goto_wall)  pending → started(벽 근접 시작) / done(이미 gap 안) / failed(벽을 못 찾음)
  wall.state  running → done(도착) / stopped / failed
"""
import json
import math
import os
import time
import urllib.error
import urllib.request

BASE_URL = os.environ.get('ARM_BASE_URL', 'http://127.0.0.1:8082').rstrip('/')
NAV_MOVING = ('backing', 'sending', 'active', 'canceling')
NAV_FAILED = ('failed', 'aborted', 'canceled', 'rejected', 'unavailable')
DRIVE_TIMEOUT_S = 240.
GAP_LIMITS = (0.02, 0.5)         # map_web /api/wall/config의 gap 허용 범위(m)
MODES = ('precise', 'wall')
# 마무리 정렬: Nav2는 장애물 옆 라벨(책상 앞 등)을 비용이 낮은 곳으로 옮겨 가고(최대 1.5 m) 방향도 ±0.25 rad까지 도착으로 친다.
XY_TOL = 0.03                    # 라벨 위치 허용 오차(m)
YAW_TOL_DEG = 2.0                # 라벨 방향 허용 오차(°)
FINE_MAX_M = 0.6                 # 이보다 멀면 마무리 정렬을 하지 않는다(Nav2가 엉뚱한 곳에 섰다)
FINE_SPEED = (0.03, 0.06)        # 직진 속도 범위(m/s): 남은 거리에 비례
TURN_SPEED = (0.10, 0.35)        # 회전 속도 범위(rad/s)
CLEAR_M = 0.03                   # 진행 방향 차체 끝에서 이만큼 안에 라이다 점이 있으면 멈춘다
TICK_S = 0.1                     # map_web 주행 명령은 0.5초만 유효하다 -> 0.1초마다 다시 보낸다
# 2026-10-27 사용자: 책상에 정확히 안 선다. 지도 위치는 map_web 0.2초 갱신 + HTTP 0.1초라 최대 약 0.3초 늦다(0.35 rad/s면 6°).
# 늦은 만큼 지나치므로, 남은 오차가 '지금 속도 × 지연' 안이면 미리 멈춘다. 목표 근처 회전은 가장 느린 속도로만 한다.
POSE_LAG_S = (0.15, 0.4)         # 지연 추정 범위(초): pose.age + 한 틱
SLOW_TURN_RAD = math.radians(10)
# AMCL은 차가 움직여야 라이다로 위치를 고친다. 마무리 정렬은 몇 cm씩만 움직여 거의 갱신되지 않아(예전 10cm·8.6°마다)
# 오도메트리만 보고 맞추고, 마지막 확인도 그 위치로 해서 '오차 1cm'라고 하면서 실제로는 어긋날 수 있었다.
# 그래서 판단하기 전마다 멈춰서 AMCL에 제자리 갱신을 요청하고(map_web /api/amcl/settle), 새 추정이 온 뒤의 위치를 쓴다.
SETTLE_WAIT_S = 2.5              # 새 AMCL 추정을 기다리는 최대 시간
SETTLE_TF_S = 0.3                # 새 추정이 지도 위치(map->odom TF, 0.2초 갱신)에 반영될 때까지


class BaseError(RuntimeError):
    """map_web이 거절했거나 닿지 않는다. code: reason_code."""

    def __init__(self, message, code='base_unavailable'):
        super().__init__(message)
        self.code = code


def _call(method, path, body=None, url=None, timeout_s=5.):
    request = urllib.request.Request((url or BASE_URL) + path, method=method,
                                     data=None if body is None else json.dumps(body).encode(),
                                     headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            return json.loads(response.read() or b'{}')
    except urllib.error.HTTPError as exc:
        try:
            message = json.loads(exc.read() or b'{}').get('error')
        except ValueError:
            message = None
        raise BaseError(message or f'주행 서버 오류 HTTP {exc.code}', 'label_not_found' if exc.code == 404 else 'base_refused') from None
    except (urllib.error.URLError, OSError, ValueError) as exc:
        raise BaseError(f'주행 서버(map_web, {url or BASE_URL})에 연결할 수 없습니다. 팀 주행 프로그램이 켜져 있는지 확인하세요. ({exc})') from None


def base_state(url=None):
    """위치·방향, 라벨 목록(가까운 순), 이동·벽 근접 상태, 드라이버 연결. 차는 움직이지 않는다."""
    state = _call('GET', '/api/state', url=url)
    drive = _call('GET', '/api/drive', url=url)
    pose = state.get('pose')
    on_map = state.get('frame') == 'map' and pose is not None
    labels = []
    for label in state.get('labels') or []:
        row = dict(id=label.get('id'), name=label.get('name'), reachable=label.get('cost') is None or label.get('cost') < 99)
        if on_map:
            row['dist_m'] = round(math.hypot(label['x'] - pose['x'], label['y'] - pose['y']), 2)
        labels.append(row)
    if on_map:
        labels.sort(key=lambda row: row['dist_m'])
    nav, wall = state.get('nav') or {}, state.get('wall') or {}
    return dict(ok=True, frame=state.get('frame'),
                pose=None if not pose else dict(x=round(pose['x'], 3), y=round(pose['y'], 3), yaw_deg=round(math.degrees(pose['yaw']), 1)),
                labels=labels, driver_connected=drive.get('driver_connected'), moving=bool(drive.get('moving')),
                odom=drive.get('odom'), nav=dict(state=nav.get('state'), server=nav.get('server'), message=nav.get('message'),
                                                 target=(nav.get('target') or {}).get('name'), distance=nav.get('distance')),
                wall=dict(state=wall.get('state'), message=wall.get('message'), gap=wall.get('gap')))


def find_label(name, labels):
    """말한 장소 이름 → 라벨 하나. 정확히 같은 이름이 먼저, 없으면 이름이 들어 있는 것. 없거나 여럿이면 BaseError."""
    name = (name or '').strip()
    exact = [label for label in labels if label.get('name') == name]
    part = exact or [label for label in labels if name and (name in label.get('name', '') or label.get('name', '') in name)]
    if len(part) == 1:
        return part[0]
    known = ', '.join(label.get('name', '') for label in labels) or '없음'
    if not part:
        raise BaseError(f"'{name}'라는 장소가 지도에 없습니다. 저장된 장소: {known}", 'label_not_found')
    raise BaseError(f"'{name}'에 맞는 장소가 여럿입니다({', '.join(label['name'] for label in part)}). 이름을 정확히 말해 주세요.", 'label_not_found')


def goto_label_wall(name, gap=None, url=None, on_phase=None, stopped=lambda: False, timeout_s=DRIVE_TIMEOUT_S, sleep=time.sleep):
    """라벨로 이동한 뒤 벽에 맞춰 gap(m) 앞까지 붙는다. 끝날 때까지 기다린다. 차가 움직인다.
    on_phase(raw, 문구): 진행 단계. stopped(): True가 되면 이동을 취소한다. 돌려주는 것: dict(ok, reason, reason_code, label, note)."""
    label = find_label(name, base_state(url)['labels'])
    if gap is not None:
        if not GAP_LIMITS[0] <= float(gap) <= GAP_LIMITS[1]:
            raise BaseError(f'gap은 {GAP_LIMITS[0]}~{GAP_LIMITS[1]} m여야 합니다.', 'bad_request')
        _call('POST', '/api/wall/config', {'gap': float(gap)}, url=url)
    started = time.time()
    _call('POST', '/api/nav/goto_wall', {'id': label['id']}, url=url)
    shown = None

    def phase(raw, text):
        nonlocal shown
        if on_phase and (raw, text) != shown:
            shown = (raw, text)
            on_phase(raw, text)

    phase('drive_start', f"{label['name']}으로 출발")
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if stopped():
            cancel(url)
            return dict(ok=False, reason='cancelled', reason_code='cancelled', label=label['name'])
        sleep(.5)
        state = _call('GET', '/api/state', url=url)
        nav, wall = state.get('nav') or {}, state.get('wall') or {}
        target = (nav.get('label') or nav.get('target') or {}).get('id')
        if target not in (None, label['id']) or (nav.get('started') or started) < started - 2:
            continue                                     # 아직 우리 목표가 반영되지 않았다
        status = nav.get('state')
        if status == 'backing':
            phase('drive_backing', '방향을 돌리려고 후진하는 중')
        elif status in NAV_MOVING:
            distance = nav.get('distance')
            phase('drive_active', f"{label['name']}으로 이동 중" + (f' (남은 {distance:.1f} m)' if isinstance(distance, (int, float)) else ''))
        elif status in NAV_FAILED:
            return dict(ok=False, reason=f"{label['name']}으로 가지 못했습니다: {nav.get('message') or status}",
                        reason_code='drive_failed', label=label['name'])
        elif status == 'succeeded':
            then = nav.get('then_state')
            if then in (None, 'pending'):
                phase('drive_arrived', f"{label['name']} 도착, 벽 찾는 중")
            elif then == 'done':
                return dict(ok=True, reason=None, reason_code=None, label=label['name'], note='도착(이미 벽 앞)')
            elif then == 'failed':
                return dict(ok=False, reason=f"{label['name']}까지는 왔지만 벽에 맞춰 붙지 못했습니다: {wall.get('message') or '정면 벽을 못 찾음'}",
                            reason_code='wall_failed', label=label['name'])
            elif wall.get('state') == 'running':
                phase('drive_wall', '벽과 수직으로 맞춰 앞으로 붙는 중')
            elif wall.get('state') == 'done':
                return dict(ok=True, reason=None, reason_code=None, label=label['name'], note=wall.get('message'))
            elif wall.get('state') in ('failed', 'stopped'):
                return dict(ok=False, reason=f"{label['name']}까지는 왔지만 벽에 붙다가 멈췄습니다: {wall.get('message') or wall.get('state')}",
                            reason_code='wall_failed', label=label['name'])
    cancel(url)
    return dict(ok=False, reason=f"{label['name']}으로 {timeout_s:.0f}초 안에 도착하지 못해 멈췄습니다.", reason_code='drive_failed', label=label['name'])


def cancel(url=None):
    """이동·벽 근접을 모두 멈춘다(map_web의 취소·정지). 실패해도 예외를 내지 않는다."""
    for path in ('/api/nav/cancel', '/api/wall/stop', '/api/drive/stop'):
        try:
            _call('POST', path, {}, url=url, timeout_s=2.)
        except BaseError:
            pass


def where_reply(state):
    """'지금 어디야?'에 대한 짧은 답."""
    if state.get('frame') != 'map' or not state.get('pose'):
        where = '지도 위 위치를 아직 모릅니다(내비게이션이 꺼져 있거나 위치를 못 잡음).'
    else:
        near = state['labels'][0] if state['labels'] else None
        where = f"지도 위치 ({state['pose']['x']:.2f}, {state['pose']['y']:.2f}) m, 방향 {state['pose']['yaw_deg']:.0f}°" + \
            (f". 가장 가까운 장소는 {near['name']}({near['dist_m']:.1f} m)" if near and 'dist_m' in near else '')
    places = ', '.join(label['name'] for label in state.get('labels') or []) or '없음'
    moving = ' 지금 이동 중입니다.' if state.get('nav', {}).get('state') in NAV_MOVING or state.get('moving') else ''
    return f'{where}. 갈 수 있는 장소: {places}.{moving}'


def short_status(state):
    """'상태 어때?'와 웹 위쪽 표시에 쓰는 한 줄: 이동 중이면 목적지와 남은 거리, 아니면 가장 가까운 장소."""
    nav = state.get('nav') or {}
    if nav.get('state') in NAV_MOVING:
        left = nav.get('distance')
        return f"차: {nav.get('target') or '목적지'}으로 이동 중" + (f' (남은 {left:.1f} m)' if isinstance(left, (int, float)) else '')
    if state.get('frame') != 'map' or not state.get('pose'):
        return '차: 지도 위 위치 모름'
    near = next((label for label in state.get('labels') or [] if 'dist_m' in label), None)
    return f"차: {near['name']} 근처 ({near['dist_m']:.1f} m)" if near else '차: 저장된 장소 없음'


# ---------------------------------------------------------------------------- 정확한 도착
def _wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _drive_cmd(linear, angular, url):
    _call('POST', '/api/drive', {'linear': round(linear, 3), 'angular': round(angular, 3)}, url=url, timeout_s=2.)


def _stop(url):
    try:
        _call('POST', '/api/drive/stop', {}, url=url, timeout_s=2.)
    except BaseError:
        pass


def _pose_and_scan(url, with_lag=False):
    state = _call('GET', '/api/state', url=url, timeout_s=2.)
    if state.get('frame') != 'map' or not state.get('pose'):
        raise BaseError('지도 위 위치를 몰라 라벨에 정확히 맞출 수 없습니다(내비게이션·위치 추정이 켜져 있는지 확인).', 'drive_failed')
    pose = state['pose']
    out = (pose['x'], pose['y'], pose['yaw']), state.get('scan') or []
    if not with_lag:
        return out
    age = pose.get('age') if isinstance(pose.get('age'), (int, float)) else 0.
    return (*out, min(POSE_LAG_S[1], max(POSE_LAG_S[0], age + TICK_S)))


def _settle_amcl(url, sleep):
    """멈춘 채로 AMCL 제자리 갱신을 요청하고 새 추정이 올 때까지 기다린다. 기다렸으면 True.
    map_web이 이 기능을 모르거나(옛 버전) AMCL 정보를 주지 않으면 잠깐만 서서 기다린다."""
    _stop(url)
    asked = time.time()
    try:
        _call('POST', '/api/amcl/settle', {}, url=url, timeout_s=2.)
    except BaseError:
        sleep(SETTLE_TF_S)
        return False
    deadline = time.monotonic() + SETTLE_WAIT_S
    while time.monotonic() < deadline:
        sleep(TICK_S * 2)
        state = _call('GET', '/api/state', url=url, timeout_s=2.)
        if 'amcl' not in state:
            break
        amcl = state.get('amcl') or {}
        if isinstance(amcl.get('received'), (int, float)) and amcl['received'] > asked:
            sleep(SETTLE_TF_S)
            return True
    sleep(SETTLE_TF_S)
    return False


def _blocked(pose, scan, robot, direction):
    """direction: +1 앞으로, -1 뒤로, 0 제자리 회전. 부딪힐 라이다 점이 있으면 그 거리(m), 없으면 None."""
    x, y, yaw = pose
    c, s_ = math.cos(yaw), math.sin(yaw)
    hits = []
    for px, py in scan:
        fx, fy = (px - x) * c + (py - y) * s_, -(px - x) * s_ + (py - y) * c      # 로봇 기준(앞 +x, 왼쪽 +y)
        if direction == 0:
            if math.hypot(fx, fy) < robot['radius']:
                hits.append(math.hypot(fx, fy))
        elif abs(fy) < robot['radius']:
            edge = robot['front'] if direction > 0 else robot['back']
            ahead = fx * direction
            if 0 < ahead - edge < CLEAR_M:
                hits.append(ahead - edge)
    return min(hits) if hits else None


def _turn_to(target_yaw, tol, url, robot, stopped, sleep, timeout_s=15.):
    deadline = time.monotonic() + timeout_s
    turning = 0.                                         # 지금 보내고 있는 회전 속도(rad/s)
    while time.monotonic() < deadline:
        if stopped():
            return 'cancelled'
        pose, scan, lag = _pose_and_scan(url, with_lag=True)
        error = _wrap(target_yaw - pose[2])
        if abs(error) <= tol + abs(turning) * lag:      # 늦게 본 위치만큼 이미 더 돌았다
            _stop(url)
            return None
        if _blocked(pose, scan, robot, 0) is not None:
            _stop(url)
            return '제자리 회전하면 주변 물체에 닿을 수 있어 멈췄습니다'
        speed = TURN_SPEED[0] if abs(error) < SLOW_TURN_RAD else min(TURN_SPEED[1], max(TURN_SPEED[0], 1.2 * abs(error)))
        turning = math.copysign(speed, error)
        _drive_cmd(0., turning, url)
        sleep(TICK_S)
    _stop(url)
    return '회전이 제시간에 끝나지 않았습니다'


def _move_to(target, heading, direction, url, robot, stopped, sleep, timeout_s=20.):
    """heading 방향을 유지하며 direction(+1 앞, -1 뒤)으로 target에 가장 가까워질 때까지 직진한다."""
    deadline = time.monotonic() + timeout_s
    ux, uy = math.cos(heading) * direction, math.sin(heading) * direction    # 실제로 움직이는 방향
    speed = 0.
    while time.monotonic() < deadline:
        if stopped():
            return 'cancelled'
        pose, scan, lag = _pose_and_scan(url, with_lag=True)
        remaining = (target[0] - pose[0]) * ux + (target[1] - pose[1]) * uy  # 진행 방향으로 남은 거리
        if remaining <= XY_TOL / 2 + speed * lag:                            # 늦게 본 위치만큼 이미 더 갔다
            _stop(url)
            return None
        gap = _blocked(pose, scan, robot, direction)
        if gap is not None:
            _stop(url)
            return f'진행 방향 {gap * 100:.0f} cm 앞에 물체가 있어 멈췄습니다'
        speed = min(FINE_SPEED[1], max(FINE_SPEED[0], 0.6 * remaining))
        steer = max(-0.15, min(0.15, 1.5 * _wrap(heading - pose[2])))       # 곧게 가도록 방향만 살짝 보정
        _drive_cmd(speed * direction, steer, url)
        sleep(TICK_S)
    _stop(url)
    return '마무리 이동이 제시간에 끝나지 않았습니다'


def fine_align(label, url=None, on_phase=None, stopped=lambda: False, sleep=time.sleep):
    """지금 위치에서 라벨의 x, y, 방향으로 천천히 맞춘다(map_web /api/drive). 돌려주는 것: dict(ok, reason, reason_code, note)."""
    phase = on_phase or (lambda raw, text: None)
    info = _call('GET', '/api/get_map_info', url=url)
    robot = {'radius': 0.26, 'front': 0.16, 'back': 0.16, **(info.get('robot') or {})}      # map_web 값이 있으면 그것
    target = (label['x'], label['y'])
    try:
        for _ in range(3):                               # 회전·직진·방향 맞춤 뒤 남은 오차를 한두 번 더 줄인다
            _settle_amcl(url, sleep)                     # 판단은 멈춘 채 새로 받은 AMCL 위치로
            pose, _ = _pose_and_scan(url)
            dx, dy = target[0] - pose[0], target[1] - pose[1]
            distance = math.hypot(dx, dy)
            yaw_error = abs(_wrap(label['yaw'] - pose[2]))
            if distance <= XY_TOL and yaw_error <= math.radians(YAW_TOL_DEG):
                break
            if distance > FINE_MAX_M:
                return dict(ok=False, reason_code='drive_failed',
                            reason=f"{label['name']} 라벨에서 {distance:.2f} m 떨어진 곳에 섰습니다(마무리 정렬 한도 {FINE_MAX_M} m). 지도 위치(AMCL)를 다시 잡아 보세요.")
            if distance > XY_TOL:
                bearing = math.atan2(dy, dx)
                forward = abs(_wrap(bearing - pose[2])) <= abs(_wrap(bearing + math.pi - pose[2]))   # 덜 도는 쪽(앞으로 또는 뒤로)
                heading = bearing if forward else _wrap(bearing + math.pi)
                phase('fine_turn', f"{label['name']} 라벨 위치로 방향 맞추는 중 ({distance * 100:.0f} cm 남음)")
                problem = _turn_to(heading, math.radians(4), url, robot, stopped, sleep)
                if not problem:
                    phase('fine_move', f"{label['name']} 라벨 위치로 {'전진' if forward else '후진'} ({distance * 100:.0f} cm)")
                    problem = _move_to(target, heading, 1 if forward else -1, url, robot, stopped, sleep)
                if problem:
                    return dict(ok=False, reason=problem, reason_code='cancelled' if problem == 'cancelled' else 'drive_failed')
            phase('fine_yaw', f"{label['name']} 라벨 방향 맞추는 중")
            problem = _turn_to(label['yaw'], math.radians(YAW_TOL_DEG), url, robot, stopped, sleep)
            if problem:
                return dict(ok=False, reason=problem, reason_code='cancelled' if problem == 'cancelled' else 'drive_failed')
        _settle_amcl(url, sleep)                         # 마지막 확인도 새 AMCL 위치로(예전에는 갱신 안 된 위치라 늘 맞다고 나왔다)
        pose, _ = _pose_and_scan(url)
        off = math.hypot(target[0] - pose[0], target[1] - pose[1])
        yaw_off = abs(math.degrees(_wrap(label['yaw'] - pose[2])))
        note = f"{label['name']} 라벨 위치 오차 {off * 100:.1f} cm, 방향 {yaw_off:.1f}°"
        if off > XY_TOL * 1.5 or yaw_off > YAW_TOL_DEG * 1.5:
            return dict(ok=False, reason=f'라벨에 정확히 맞추지 못했습니다: {note}', reason_code='drive_imprecise', note=note)
        return dict(ok=True, reason=None, reason_code=None, note=note)
    finally:
        _stop(url)


def goto_label_precise(name, url=None, on_phase=None, stopped=lambda: False, timeout_s=DRIVE_TIMEOUT_S, sleep=time.sleep):
    """라벨을 만든 그 자리(위치·방향)로 간다: Nav2로 근처까지(/api/nav/goto) → fine_align. 차가 움직인다."""
    state = _call('GET', '/api/state', url=url)
    raw = {label['id']: label for label in state.get('labels') or []}
    label = raw[find_label(name, base_state(url)['labels'])['id']]
    phase = on_phase or (lambda raw_phase, text: None)
    started = time.time()
    _call('POST', '/api/nav/goto', {'id': label['id']}, url=url)
    phase('drive_start', f"{label['name']}으로 출발")
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if stopped():
            cancel(url)
            return dict(ok=False, reason='cancelled', reason_code='cancelled', label=label['name'])
        sleep(.5)
        nav = _call('GET', '/api/state', url=url).get('nav') or {}
        target = (nav.get('label') or nav.get('target') or {}).get('id')
        if target not in (None, label['id']) or (nav.get('started') or started) < started - 2:
            continue
        status = nav.get('state')
        if status == 'backing':
            phase('drive_backing', '방향을 돌리려고 후진하는 중')
        elif status in NAV_MOVING:
            distance = nav.get('distance')
            phase('drive_active', f"{label['name']}으로 이동 중" + (f' (남은 {distance:.1f} m)' if isinstance(distance, (int, float)) else ''))
        elif status in NAV_FAILED:
            return dict(ok=False, reason=f"{label['name']}으로 가지 못했습니다: {nav.get('message') or status}",
                        reason_code='drive_failed', label=label['name'])
        elif status == 'succeeded':
            result = fine_align(label, url, on_phase, stopped, sleep)
            return dict(result, label=label['name'])
    cancel(url)
    return dict(ok=False, reason=f"{label['name']}으로 {timeout_s:.0f}초 안에 도착하지 못해 멈췄습니다.", reason_code='drive_failed', label=label['name'])


def drive_to(name, mode='precise', gap=None, url=None, on_phase=None, stopped=lambda: False, timeout_s=DRIVE_TIMEOUT_S, sleep=time.sleep):
    """precise(기본): 라벨의 정확한 위치·방향으로. wall: 라벨로 간 뒤 앞 벽과 수직으로 gap까지 붙는다."""
    if mode == 'wall':
        return goto_label_wall(name, gap=gap, url=url, on_phase=on_phase, stopped=stopped, timeout_s=timeout_s, sleep=sleep)
    if mode != 'precise':
        raise BaseError(f'mode는 {MODES} 중 하나입니다.', 'bad_request')
    return goto_label_precise(name, url=url, on_phase=on_phase, stopped=stopped, timeout_s=timeout_s, sleep=sleep)
