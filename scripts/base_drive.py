"""팀 주행 서버(ros2_slam test/map_web.py, 같은 Pi의 8081)의 REST API로 차를 움직인다. 쓰는 기능은 둘뿐이다.

  base_state       GET /api/state, /api/drive   위치·방향, 라벨 목록, Nav2/벽 근접 상태
  goto_label_wall  POST /api/nav/goto_wall      라벨로 이동한 뒤 정면 벽과 수직으로 맞추고 gap(m) 앞까지 전진

주소는 ARM_BASE_URL(기본 http://127.0.0.1:8081). map_web 쪽 코드는 건드리지 않는다.
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

BASE_URL = os.environ.get('ARM_BASE_URL', 'http://127.0.0.1:8081').rstrip('/')
NAV_MOVING = ('backing', 'sending', 'active', 'canceling')
NAV_FAILED = ('failed', 'aborted', 'canceled', 'rejected', 'unavailable')
DRIVE_TIMEOUT_S = 240.
GAP_LIMITS = (0.02, 0.5)         # map_web /api/wall/config의 gap 허용 범위(m)


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

    phase('drive_start', f"{label['name']}(으)로 출발")
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
            phase('drive_active', f"{label['name']}(으)로 이동 중" + (f' (남은 {distance:.1f} m)' if isinstance(distance, (int, float)) else ''))
        elif status in NAV_FAILED:
            return dict(ok=False, reason=f"{label['name']}(으)로 가지 못했습니다: {nav.get('message') or status}",
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
    return dict(ok=False, reason=f"{label['name']}(으)로 {timeout_s:.0f}초 안에 도착하지 못해 멈췄습니다.", reason_code='drive_failed', label=label['name'])


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
        return f"차: {nav.get('target') or '목적지'}(으)로 이동 중" + (f' (남은 {left:.1f} m)' if isinstance(left, (int, float)) else '')
    if state.get('frame') != 'map' or not state.get('pose'):
        return '차: 지도 위 위치 모름'
    near = next((label for label in state.get('labels') or [] if 'dist_m' in label), None)
    return f"차: {near['name']} 근처 ({near['dist_m']:.1f} m)" if near else '차: 저장된 장소 없음'
