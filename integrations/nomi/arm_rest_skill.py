"""NOMI(동료 LLM 저장소) Skill 어댑터: 로봇팔 REST API(scripts/arm_api.py)를 NOMI ToolRegistry가 부르는 Skill로 감싼다.

- ROS·외부 패키지 없이 표준 라이브러리(urllib)만 쓴다. NOMI의 nomi_agent/skills/ 아래에 그대로 복사해 쓴다.
- 돌려주는 값은 NOMI Skill 형식 {'success', 'message', 'data'}이다.
- 움직이는 작업은 끝날 때까지 /tasks/<id>를 기다린 뒤, 로봇 쪽 결과가 성공일 때만 data.completion_verified=True로 둔다
  (NOMI의 requires_completion_evidence 검사와 맞춘다). LLM은 관절값을 보내지 않고 물체 이름·칸 번호만 넘긴다.
- 설정: 환경 변수 ARM_API_URL(기본 http://127.0.0.1:8000), ARM_API_TOKEN(서버 --token 값).
"""
import json
import os
import threading
import time
import urllib.error
import urllib.request

OBJECTS = ('white_box', 'red_cube', 'tape_measure', 'unknown')
DEFAULT_SLOT = {'white_box': 1, 'red_cube': 2, 'tape_measure': 3}    # 칸을 말하지 않았을 때 넣는 칸(메뉴 5번에 그대로 보낸다)
TIDY_ORDER = ('white_box', 'red_cube', 'tape_measure')                # 책상 정리하기 순서
MENU_TIDY, MENU_PICK, MENU_HOME, MENU_UNKNOWN = 4, 5, 6, 7           # REST 주소 번호(/api/menu/N). 화면 메뉴 번호와는 다르다
# 결과 이유 코드 → 말할 문장(원문이 영어이거나 비어 있을 때 쓴다. TTS가 그대로 읽는다)
REASON_TEXT = {
    'cancelled': '중간에 정지했습니다',
    'busy': '로봇팔이 다른 작업을 하는 중입니다',
    'camera_busy': '카메라 창이 켜져 있어 카메라를 쓸 수 없습니다',
    'camera_error': '카메라로 사진을 찍지 못했습니다',
    'object_not_found': '물체를 찾지 못했습니다',
    'multiple_new_objects': '새 물체는 한 번에 하나만 놓아 주세요',
    'tray_not_found': '트레이를 찾지 못했습니다',
    'tray_moved': '트레이 위치가 바뀌었습니다',
    'slot_too_small': '칸에 여유가 부족합니다',
    'unreachable': '팔이 닿지 않는 위치입니다',
    'path_blocked': '운반 경로에 다른 물체가 있어 부딪힐 수 있습니다',
    'object_moved': '물체 위치가 바뀌었습니다',
    'not_held': '물체를 집지 못했습니다',
    'torque_off': '서보 토크가 꺼져 있습니다',
    'arm_state_unknown': '팔 상태를 읽지 못했습니다',
    'arm_not_folded': '팔이 접혀 있지 않습니다',
    'device_absent': '장치 연결을 확인해 주세요',
    'base_moving': '차가 움직이는 중입니다',
    'unauthorized': '로봇팔 서버 비밀번호(토큰)가 맞지 않습니다',
    'connection_error': '로봇팔 서버에 연결하지 못했습니다',
}
ALIASES = {
    'white_box': ('white_box', '흰 상자', '흰상자', '하얀 상자', '하얀상자', '흰 박스', '박스', '상자'),
    'red_cube': ('red_cube', '빨간 정육면체', '빨간정육면체', '빨간 큐브', '빨간큐브', '큐브', '정육면체'),
    'tape_measure': ('tape_measure', '줄자'),
    'unknown': ('unknown', '새 물체', '모르는 물체', '등록 안 된 물체'),
}
NAMES = {'white_box': '흰 상자', 'red_cube': '빨간 정육면체', 'tape_measure': '줄자', 'unknown': '새 물체'}


def say_reason(reason, code=None):
    """결과 이유를 말할 문장으로. 한국어 원문은 그대로, 영어·빈 원문은 코드로 고른 문장."""
    text = str(reason or '').strip()
    if text and any('가' <= c <= '힣' for c in text):
        return text
    return REASON_TEXT.get(code) or REASON_TEXT.get(text) or text or '알 수 없는 이유'


def resolve_objects(names, registered=()):
    """책상 정리하기에서 고른 물체들. 리스트나 '흰 상자, 줄자' 같은 쉼표 문자열. (등록 이름 목록, 모르는 이름) — 비면 ([], None).
    registered: 서버에 등록된 물체 [{label, name}] (GET /api/objects). 순서는 기본 세 물체 → 등록 물체."""
    if isinstance(names, str):
        names = [n for n in names.replace('、', ',').split(',') if n.strip()]
    order = list(TIDY_ORDER) + [o['label'] for o in registered]
    labels = []
    for name in names or ():
        label = resolve_object(name, registered)
        if label not in order:
            return None, name
        labels.append(label)
    return [label for label in order if label in labels], None


def resolve_object(name, registered=()):
    """한국어·영어 물체 이름을 등록 이름(label)으로 바꾼다. 서버에 등록된 물체 이름·custom_N도 받는다. 모르면 None."""
    key = ''.join(str(name or '').split()).casefold()
    for label, words in ALIASES.items():
        if any(key == ''.join(w.split()).casefold() for w in words):
            return label
    for o in registered:
        if key in (''.join(o['name'].split()).casefold(), o['label'].casefold()):
            return o['label']
    return None


class ArmRestSkill:
    POLL_S = 0.5
    OBJECTS_CACHE_S = 10.0      # 등록 물체 목록(GET /api/objects)을 이 시간 동안 다시 묻지 않는다
    POLL_ERRORS_MAX = 20        # 결과 조회가 연달아 이만큼(약 10초) 실패하면 서버를 잃은 것으로 보고 정지를 보낸다
    STOP_GUARD_S = 3.0          # 정지 직후 이 시간 안에 들어온 움직이는 명령은 보내지 않는다(호출 직전에 누른 비상정지가 지워지지 않게)

    def __init__(self, node=None, base_url=None, token=None, http_timeout=10.0):
        self.node = node
        self.base_url = (base_url or os.environ.get('ARM_API_URL') or 'http://127.0.0.1:8000').rstrip('/')
        self.token = token if token is not None else os.environ.get('ARM_API_TOKEN', '')
        self.http_timeout = http_timeout
        self._cancel = threading.Event()     # 지금 기다리는 작업의 취소 표시(작업마다 새로 만든다)
        self._stopped_at = None              # 마지막 stop() 시각(time.monotonic)
        self._objects = (None, [])           # (받은 시각, 등록 물체 [{label, name, slot}])

    # ---- HTTP ----
    def _request(self, method, path, body=None, timeout=None):
        headers = {'Content-Type': 'application/json'}
        if self.token:
            headers['Authorization'] = f'Bearer {self.token}'
        data = json.dumps(body).encode() if body is not None else (b'{}' if method == 'POST' else None)
        request = urllib.request.Request(self.base_url + path, data=data, method=method, headers=headers)
        try:
            with urllib.request.urlopen(request, timeout=timeout or self.http_timeout) as response:
                return response.status, json.loads(response.read() or b'{}')
        except urllib.error.HTTPError as error:
            try:
                return error.code, json.loads(error.read() or b'{}')
            except ValueError:
                return error.code, {}
        except (urllib.error.URLError, OSError, ValueError) as exc:
            return None, {'reason_code': 'connection_error', 'reason': f'{type(exc).__name__}: {exc}'}

    @staticmethod
    def _fail(message, **data):
        return {'success': False, 'message': message, 'data': {'completion_verified': False, **data}}

    def _registered(self):
        """서버에 등록된 물체 [{label, name, slot}]. 서버에 못 닿으면 마지막으로 받은 목록(없으면 [])."""
        stamp, rows = self._objects
        if stamp is not None and time.monotonic() - stamp < self.OBJECTS_CACHE_S:
            return rows
        status, body = self._request('GET', '/api/objects', timeout=5)
        if status == 200:
            rows = [{k: o.get(k) for k in ('label', 'name', 'slot')} for o in body.get('objects', []) if o.get('registered')]
            self._objects = (time.monotonic(), rows)
        return rows

    def _name(self, label):
        return NAMES.get(label) or next((o['name'] for o in self._registered() if o['label'] == label), label)

    def _choices_text(self):
        return ', '.join(['흰 상자', '빨간 정육면체', '줄자'] + [o['name'] for o in self._registered()])

    # ---- 읽기 전용 ----
    def look_at_table(self) -> dict:
        """위 카메라로 지금 장면을 찍어 보이는 물체와 쓸 수 있는 메뉴를 돌려준다(약 4초, 로봇은 움직이지 않음).
        사진(base64)은 LLM 문맥이 커지지 않게 빼고 image_url만 남긴다."""
        status, body = self._request('POST', '/api/image', timeout=40)
        if status != 200 or not body.get('ok'):
            return self._fail(f"장면을 찍지 못했습니다: {say_reason(body.get('reason'), body.get('reason_code')) if body else status}",
                              error_code=body.get('reason_code', 'camera_error'))
        objects = [{k: o.get(k) for k in ('label', 'name', 'position_mm', 'pickable')} for o in body.get('objects', [])]
        in_tray = [{k: o.get(k) for k in ('label', 'name', 'slot')} for o in body.get('in_tray', [])]
        tray_found = body.get('tray_found')
        # 메뉴 번호(available_menus)는 LLM에게 뜻이 없어 빼고 말로 알린다(화면 메뉴 번호와도 다르다)
        parts = [f"작업대 물체: {', '.join(o['name'] for o in objects) or '없음'}"]
        stuck = [o['name'] for o in objects if o.get('pickable') is False]
        if stuck:
            parts.append(f"지금 집을 수 없음: {', '.join(stuck)}")
        if in_tray:
            parts.append('트레이에 이미 있음: ' + ', '.join(f"{o['name']}({o['slot']}번 칸)" for o in in_tray))
        if tray_found is False:
            parts.append('트레이를 찾지 못했습니다')
        return {'success': True, 'message': '. '.join(parts) + '.',
                'data': {'objects': objects, 'in_tray': in_tray, 'tray_found': tray_found,
                         'captured_at': body.get('captured_at'),
                         'image_url': self.base_url + body.get('image_url', '/api/image/latest.jpg'),
                         'completion_verified': True}}

    def get_arm_status(self) -> dict:
        status, body = self._request('GET', '/status')
        if status != 200:
            code = body.get('reason_code') or ('unauthorized' if status == 401 else 'connection_error')
            return self._fail(f"로봇팔 상태를 읽지 못했습니다: {say_reason(body.get('reason'), code)}", error_code=code)
        home = {True: 'HOME 자세입니다', False: 'HOME 자세가 아닙니다'}.get(body.get('arm_home'), 'HOME 자세인지 모릅니다(중단·실패 뒤)')
        if body.get('state') == 'busy':
            step = body.get('phase_label')
            message = f"로봇팔 작업 중" + (f"(지금 단계: {step})" if step else '') + '.'
        else:
            message = f'로봇팔 대기 중, {home}.'
        return {'success': True, 'message': message,
                'data': {**body, 'completion_verified': True}}

    # ---- 움직이는 작업 ----
    def sort_object(self, object_name: str | None = None, slot: int | None = None) -> dict:
        registered = self._registered()
        label = resolve_object(object_name, registered)
        if label is None:
            return self._fail(f"'{object_name}'은(는) 등록된 물체가 아닙니다. {self._choices_text()} 중에서 골라 주세요.",
                              error_code='unknown_object', needs_clarification=True)
        if slot is not None and (isinstance(slot, bool) or slot not in (1, 2, 3)):
            return self._fail('칸은 1, 2, 3번 중 하나입니다.', error_code='invalid_slot', needs_clarification=True)
        if label == 'unknown':
            if slot is None:
                return self._fail('새 물체를 넣을 칸(1~3번)을 알려 주세요.', error_code='missing_slot', needs_clarification=True)
            return self._run(MENU_UNKNOWN, {'slot': slot}, f'새 물체 → {slot}번 칸', timeout=300)
        if slot is None:                                          # 기본 칸도 메뉴 5번에 칸을 붙여 보낸다(메뉴 번호가 바뀌어도 덜 묶이게)
            slot = DEFAULT_SLOT.get(label) or next(o['slot'] for o in registered if o['label'] == label)
        return self._run(MENU_PICK, {'object': label, 'slot': slot}, f'{self._name(label)} → {slot}번 칸')

    def sort_all_objects(self, objects=None) -> dict:
        """책상 정리하기. objects를 주면 그 물체만(같은 순서), 없으면 책상에 있는 등록 물체 전부."""
        registered = self._registered()
        labels, bad = resolve_objects(objects, registered)
        if bad is not None:
            return self._fail(f"'{bad}'은(는) 정리할 수 있는 물체가 아닙니다. {self._choices_text()} 중에서 골라 주세요.",
                              error_code='unknown_object', needs_clarification=True)
        if labels:
            return self._run(MENU_TIDY, {'objects': labels}, '책상 정리하기(' + ', '.join(self._name(l) for l in labels) + ')',
                             timeout=200 * len(labels))
        return self._run(MENU_TIDY, None, '책상 정리하기', timeout=200 * (len(TIDY_ORDER) + len(registered)))   # 물체 하나에 약 2분

    def arm_return_home(self) -> dict:
        return self._run(MENU_HOME, None, 'HOME 복귀', timeout=90)

    def stop(self) -> bool:
        """기다리던 작업을 끊고 로봇팔에 즉시 정지를 보낸다(NOMI emergency_stop·취소에서 부른다)."""
        self._stopped_at = time.monotonic()
        self._cancel.set()
        status, body = self._request('POST', '/stop', timeout=5)
        return status == 200 and bool(body.get('ok'))

    def _run(self, number, body, title, timeout=240):
        cancel = self._cancel = threading.Event()      # 작업마다 새 표시: 앞 작업의 취소를 지우지도, 물려받지도 않는다
        if self._stopped_at is not None and time.monotonic() - self._stopped_at < self.STOP_GUARD_S:
            return self._fail(f'{title}: 방금 정지 명령이 있어 시작하지 않았습니다. 다시 시키면 실행합니다.',
                              error_code='cancelled', menu=number, request_dispatched=False)
        status, started = self._request('POST', f'/api/menu/{number}', body)
        if status != 202 or not started.get('task_id'):
            code = started.get('reason_code') or ('unauthorized' if status == 401 else 'start_failed')
            return self._fail(f"{title}을(를) 시작하지 못했습니다: {say_reason(started.get('reason'), code) if started else status}",
                              error_code=code, menu=number, request_dispatched=False)
        task_id = started['task_id']
        deadline = time.monotonic() + timeout
        task = {}
        errors = 0
        while time.monotonic() < deadline:
            if cancel.is_set():
                self._request('POST', '/stop', timeout=5)       # 정지가 작업 시작보다 먼저 서버에 닿았을 수 있어 한 번 더 보낸다
                return self._fail(f'{title}: 취소되어 정지를 보냈습니다.', error_code='cancelled', task_id=task_id,
                                  menu=number, request_dispatched=True)
            status, got = self._request('GET', f'/tasks/{task_id}')
            if status == 200:
                errors = 0
                task = got
                if task.get('state') != 'running':
                    break
            elif status == 404:
                # 서버가 다시 켜져 작업 기록이 없다: 결과를 알 수 없으니 기다리지 않는다
                self._request('POST', '/stop', timeout=5)
                return self._fail(f'{title}: 로봇팔 서버가 다시 시작되어 작업 결과를 알 수 없습니다. 팔 상태를 확인해 주세요.',
                                  error_code='task_lost', task_id=task_id, menu=number, request_dispatched=True,
                                  last_phase=task.get('phase_label'))
            else:
                errors += 1
                if errors >= self.POLL_ERRORS_MAX:
                    self._request('POST', '/stop', timeout=5)
                    return self._fail(f'{title}: 로봇팔 서버와 연결이 끊겨 정지를 보냈습니다. 팔 상태를 확인해 주세요.',
                                      error_code='connection_lost', task_id=task_id, menu=number, request_dispatched=True,
                                      last_phase=task.get('phase_label'))
            time.sleep(self.POLL_S)
        else:
            self._request('POST', '/stop', timeout=5)       # 결과를 모르는 채로 계속 움직이게 두지 않는다
            return self._fail(f'{title}: {timeout}초 안에 끝나지 않아 정지를 보냈습니다.', error_code='timeout',
                              task_id=task_id, menu=number, request_dispatched=True, last_phase=task.get('phase_label'))
        result = task.get('result') or {}
        data = {'task_id': task_id, 'menu': number, 'request_dispatched': True,
                'phases': [p.get('label') if isinstance(p, dict) else p for p in task.get('phases', [])],
                'items': task.get('items', []), 'warnings': result.get('warnings', []),
                'seconds': result.get('seconds')}
        summary = items_summary(data['items'], {o['label']: o['name'] for o in self._objects[1]})
        if result.get('unregistered_left'):                             # 등록 안 된 물체는 피해 가고 그대로 둔다
            summary = ' / '.join(p for p in (summary, f"등록 안 된 물체 {result['unregistered_left']}개는 그대로 둠") if p)
        data['unregistered_left'] = result.get('unregistered_left', 0)
        if task.get('state') == 'done' and result.get('ok'):
            note = f" {summary or result.get('note') or ''}".rstrip()       # 예: 넣음: 흰 상자 / 건너뜀(책상에 없음): 빨간 정육면체
            return {'success': True, 'message': f'{title} 완료.{note}', 'data': {**data, 'completion_verified': True}}
        code = result.get('reason_code') or 'failed'
        tail = f' ({summary})' if summary else ''                          # 책상 정리하기에서 앞 물체는 넣었는지 함께 알린다
        return self._fail(f"{title} 실패: {say_reason(result.get('reason') or task.get('state'), code)}{tail}",
                          error_code=code, **data)


def items_summary(items, names=None):
    """책상 정리하기 물체별 결과를 한 줄로. 물체별 결과가 없으면 ''."""
    groups = {'넣음': [], '실패': [], '건너뜀(책상에 없음)': []}
    for item in items or ():
        name = (names or {}).get(item.get('object')) or NAMES.get(item.get('object'), item.get('object'))
        key = '건너뜀(책상에 없음)' if item.get('skipped') else '넣음' if item.get('ok') else '실패'
        groups[key].append(name)
    return ' / '.join(f"{key}: {', '.join(names)}" for key, names in groups.items() if names)
