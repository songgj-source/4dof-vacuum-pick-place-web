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
DEFAULT_MENU = {'white_box': 1, 'red_cube': 2, 'tape_measure': 3}    # 기본 칸에 넣는 메뉴 번호
ALIASES = {
    'white_box': ('white_box', '흰 상자', '흰상자', '하얀 상자', '하얀상자', '흰 박스', '박스', '상자'),
    'red_cube': ('red_cube', '빨간 정육면체', '빨간정육면체', '빨간 큐브', '빨간큐브', '큐브', '정육면체'),
    'tape_measure': ('tape_measure', '줄자'),
    'unknown': ('unknown', '새 물체', '모르는 물체', '등록 안 된 물체'),
}
NAMES = {'white_box': '흰 상자', 'red_cube': '빨간 정육면체', 'tape_measure': '줄자', 'unknown': '새 물체'}


def resolve_object(name):
    """한국어·영어 물체 이름을 등록 이름으로 바꾼다. 모르면 None."""
    key = ''.join(str(name or '').split()).casefold()
    for label, words in ALIASES.items():
        if any(key == ''.join(w.split()).casefold() for w in words):
            return label
    return None


class ArmRestSkill:
    POLL_S = 0.5

    def __init__(self, node=None, base_url=None, token=None, http_timeout=10.0):
        self.node = node
        self.base_url = (base_url or os.environ.get('ARM_API_URL') or 'http://127.0.0.1:8000').rstrip('/')
        self.token = token if token is not None else os.environ.get('ARM_API_TOKEN', '')
        self.http_timeout = http_timeout
        self._cancel = threading.Event()

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

    # ---- 읽기 전용 ----
    def look_at_table(self) -> dict:
        """위 카메라로 지금 장면을 찍어 보이는 물체와 쓸 수 있는 메뉴를 돌려준다(약 4초, 로봇은 움직이지 않음).
        사진(base64)은 LLM 문맥이 커지지 않게 빼고 image_url만 남긴다."""
        status, body = self._request('POST', '/api/image', timeout=40)
        if status != 200 or not body.get('ok'):
            return self._fail(f"장면을 찍지 못했습니다: {body.get('reason') or body.get('reason_code') or status}",
                              error_code=body.get('reason_code', 'camera_error'))
        objects = [{k: o.get(k) for k in ('label', 'name', 'position_mm', 'pickable')} for o in body.get('objects', [])]
        seen = ', '.join(o['name'] for o in objects) or '없음'
        return {'success': True, 'message': f'보이는 물체: {seen}. 쓸 수 있는 메뉴: {body.get("available_menus")}',
                'data': {'objects': objects, 'available_menus': body.get('available_menus', []),
                         'note': body.get('note'), 'captured_at': body.get('captured_at'),
                         'image_url': self.base_url + body.get('image_url', '/api/image/latest.jpg'),
                         'completion_verified': True}}

    def get_arm_status(self) -> dict:
        status, body = self._request('GET', '/status')
        if status != 200:
            return self._fail(f"로봇팔 상태를 읽지 못했습니다: {body.get('reason') or body.get('reason_code') or status}",
                              error_code=body.get('reason_code', 'connection_error'))
        word = '작업 중' if body.get('state') == 'busy' else '대기 중'
        return {'success': True, 'message': f"로봇팔 {word}, HOME 자세: {body.get('arm_home')}",
                'data': {**body, 'completion_verified': True}}

    # ---- 움직이는 작업 ----
    def sort_object(self, object_name: str | None = None, slot: int | None = None) -> dict:
        label = resolve_object(object_name)
        if label is None:
            return self._fail(f"'{object_name}'은(는) 등록된 물체가 아닙니다. 흰 상자, 빨간 정육면체, 줄자 중에서 골라 주세요.",
                              error_code='unknown_object', needs_clarification=True)
        if slot is not None and (isinstance(slot, bool) or slot not in (1, 2, 3)):
            return self._fail('칸은 1, 2, 3번 중 하나입니다.', error_code='invalid_slot', needs_clarification=True)
        if label == 'unknown':
            if slot is None:
                return self._fail('새 물체를 넣을 칸(1~3번)을 알려 주세요.', error_code='missing_slot', needs_clarification=True)
            return self._run(7, {'slot': slot}, f'새 물체 → {slot}번 칸', timeout=300)
        if slot is None:
            return self._run(DEFAULT_MENU[label], None, f'{NAMES[label]} → {DEFAULT_MENU[label]}번 칸')
        return self._run(5, {'object': label, 'slot': slot}, f'{NAMES[label]} → {slot}번 칸')

    def sort_all_objects(self) -> dict:
        return self._run(4, None, '세 물체 모두 넣기', timeout=600)

    def arm_return_home(self) -> dict:
        return self._run(6, None, 'HOME 복귀', timeout=90)

    def stop(self) -> bool:
        """기다리던 작업을 끊고 로봇팔에 즉시 정지를 보낸다(NOMI emergency_stop·취소에서 부른다)."""
        self._cancel.set()
        status, body = self._request('POST', '/stop', timeout=5)
        return status == 200 and bool(body.get('ok'))

    def _run(self, number, body, title, timeout=240):
        self._cancel.clear()
        status, started = self._request('POST', f'/api/menu/{number}', body)
        if status != 202 or not started.get('task_id'):
            reason = started.get('reason') or started.get('reason_code') or status
            return self._fail(f'{title}을(를) 시작하지 못했습니다: {reason}',
                              error_code=started.get('reason_code', 'start_failed'), menu=number, request_dispatched=False)
        task_id = started['task_id']
        deadline = time.monotonic() + timeout
        task = {}
        while time.monotonic() < deadline:
            if self._cancel.is_set():
                return self._fail(f'{title}: 취소되어 정지를 보냈습니다.', error_code='cancelled', task_id=task_id,
                                  menu=number, request_dispatched=True)
            status, got = self._request('GET', f'/tasks/{task_id}')
            if status == 200:
                task = got
                if task.get('state') != 'running':
                    break
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
        if task.get('state') == 'done' and result.get('ok'):
            note = f" {result['note']}" if result.get('note') else ''      # 예: 책상에 없어 건너뜀: 빨간 정육면체
            return {'success': True, 'message': f'{title} 완료.{note}', 'data': {**data, 'completion_verified': True}}
        return self._fail(f"{title} 실패: {result.get('reason') or task.get('state')}",
                          error_code=result.get('reason_code') or 'failed', **data)
