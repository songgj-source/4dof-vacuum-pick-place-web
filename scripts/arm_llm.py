"""웹 채팅의 말을 로봇팔 명령으로 바꾼다(로컬 Ollama 도구 호출). 로봇을 직접 움직이지 않는다.

- 읽기만 하는 것(상태 확인, 사진 찍기)과 정지는 바로 실행한다. 정지는 늦으면 안 된다.
- 집기·HOME 복귀는 '제안'으로만 돌려준다. 웹 화면에서 사람이 [실행]을 눌러야 움직인다.
- 모델 출력은 그대로 믿지 않고 허용 목록으로 다시 검사한다(validate).

필요: Ollama가 켜져 있고 모델이 받아져 있어야 한다(ollama pull qwen3.5:9b). 추가 패키지는 필요 없다(표준 라이브러리 HTTP).
환경 변수: OLLAMA_HOST(기본 http://127.0.0.1:11434), ARM_LLM_MODEL(기본 qwen3.5:9b), ARM_LLM_TIMEOUT(초, 기본 120: 첫 요청은 모델을 올리느라 오래 걸린다).
"""
import json
import os
import re
import sys
from pathlib import Path
import urllib.error
import urllib.request

MODEL = os.environ.get('ARM_LLM_MODEL', 'qwen3.5:9b')
ROBOT_NAME = os.environ.get('ARM_ROBOT_NAME', '자비스')
MAX_TURNS = 3        # 상태·사진을 본 뒤 답하는 정도까지만 반복한다
OBJECTS = ('white_box', 'red_cube', 'tape_measure', 'unknown')

SYSTEM = f"""당신은 SO-101 4축 진공흡착 로봇팔 '{ROBOT_NAME}'입니다. 이름을 물으면 {ROBOT_NAME}라고 답하세요. 사용자의 한국어 요청을 도구 호출로 바꿉니다.
물체: white_box(흰 상자, 기본 1번 칸), red_cube(빨간 정육면체·큐브, 기본 2번 칸), tape_measure(줄자, 기본 3번 칸),
unknown(목록에 없는 새 물체: 시작 뒤 화면 안내에 따라 사람이 물체를 놓는다). 칸은 1·2·3번뿐입니다.
- 물체를 넣으라는 요청은 sort_object를 호출하세요. 칸을 말하지 않으면 slot을 빼세요(기본 칸). unknown은 칸이 꼭 필요하니 없으면 되물으세요.
- "새 물체", "새로운 물체", "모르는 물체"는 object_name "unknown"입니다.
  unknown은 시작한 뒤에 사람이 물체를 놓는 방식이라, 지금 작업대에 새 물체가 안 보여도 정상입니다. look_at_table로 확인하지 말고 칸만 있으면 바로 sort_object를 호출하세요.
- "다 넣어줘", "다 정리해줘", "책상 정리해줘", "치워줘", "분류해줘", "전부 넣어", "세 개 모두"는 되묻지 말고 바로 sort_all_objects, "팔 접어", "HOME으로", "홈으로 복귀"는 arm_return_home입니다.
- 도구를 써야 할 때는 도구 이름을 글로 설명하지 말고 실제로 호출하세요.
- "뭐 있어?", "뭘 넣을 수 있어?" 같은 질문은 look_at_table로 사진을 찍어 확인하세요. 로봇은 움직이지 않습니다.
- 상태를 물으면 get_arm_status, "멈춰", "정지"는 stop_arm을 바로 호출하세요.
- 움직이는 명령(sort_object, sort_all_objects, arm_return_home)은 호출하면 바로 실행됩니다. 사용자가 분명히 시킨 경우에만 호출하고, 한 번에 하나만 호출하세요. "넣는 중"처럼 이미 움직이는 것처럼 말하지 마세요.
- "책상으로 가", "휴지통 쪽으로 이동해"처럼 장소로 가라는 말은 move_to_place를 호출하세요(팔 작업이 아닙니다).
- 물체가 애매하면 추측하지 말고 되물으세요. 답은 한두 문장으로 짧게 하세요."""


def _tool(name, description, properties=None, required=()):
    return {'type': 'function', 'function': {'name': name, 'description': description,
            'parameters': {'type': 'object', 'properties': properties or {}, 'required': list(required)}}}


TOOLS = [
    _tool('look_at_table', '위 카메라로 지금 작업대를 찍어 보이는 물체와 쓸 수 있는 메뉴 번호를 알려 준다. 로봇은 움직이지 않는다(약 4초). '
          '사용자가 무엇이 있는지 물을 때만 부른다. 넣으라는 명령에는 부르지 말고 바로 sort_object를 쓴다.'),
    _tool('get_arm_status', '로봇팔이 작업 중인지, HOME 자세인지, 진행 단계를 읽는다. 움직이지 않는다.'),
    _tool('sort_object', '물체를 찾아 집어 트레이 칸에 넣고 HOME으로 돌아온다. 칸을 말하지 않으면 기본 칸(흰 상자 1, 정육면체 2, 줄자 3). '
          'unknown(새 물체)은 칸이 꼭 필요하고, 시작한 뒤에 사람이 물체를 놓으므로 작업대에 아직 없어도 된다.',
          {'object_name': {'type': 'string', 'enum': list(OBJECTS), 'description': '넣을 물체'},
           'slot': {'type': 'integer', 'enum': [1, 2, 3], 'description': '넣을 칸 번호. 사용자가 말했을 때만'}}, ['object_name']),
    _tool('sort_all_objects', '흰 상자 → 빨간 정육면체 → 줄자 순서로 세 물체를 모두 기본 칸에 넣는다. 앞 물체가 실패하면 멈춘다.'),
    _tool('arm_return_home', '팔을 접힌 HOME 자세로 돌려놓는다.'),
    _tool('move_to_place', '로봇 차를 지도에 저장된 장소(라벨)로 이동시킨 뒤 정면 벽(작업대) 앞에 붙인다. 팔은 HOME으로 접혀 있어야 한다. '
          '장소 이름은 사용자가 말한 그대로(예: 책상, 휴지통).', {'place': {'type': 'string', 'description': '장소 이름'}}, ('place',)),
    _tool('stop_arm', '실행 중인 동작을 즉시 멈춘다.'),
]
MOTION = {'sort_object', 'sort_all_objects', 'arm_return_home', 'move_to_place'}
NAMES = {'white_box': '흰 상자', 'red_cube': '빨간 정육면체', 'tape_measure': '줄자', 'unknown': '새 물체'}
DEFAULT_SLOT = {'white_box': 1, 'red_cube': 2, 'tape_measure': 3}

# ---- 규칙으로 먼저 처리하는 분명한 명령(9B 모델이 흔들려도 결과가 같게) ----
# 물체·칸 해석은 터미널 명령(run_sort.py --command)과 같은 sorting.sort_job.parse_command를 쓴다.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from sorting.sort_job import parse_command          # noqa: E402
import base_drive                                   # noqa: E402  팀 주행 서버(map_web)

_SYNONYMS = (('빨간 큐브', '빨간 정육면체'), ('빨간큐브', '빨간 정육면체'), ('큐브', '정육면체'), ('새로운 물체', '새 물체'),
             ('모르는 물체', '새 물체'), ('등록 안 된 물체', '새 물체'), ('새물체', '새 물체'))
_STOP = re.compile(r'^\s*(멈춰|멈춰줘|멈춰라|정지|정지해|정지해줘|그만|그만해|스톱|stop)\s*[.!]*\s*$', re.I)
_PUT = re.compile(r'넣|정리|옮겨|분류|담아|치워')
_TIDY = re.compile(r'정리|치워|분류')     # 물체 이름 없이 "책상 정리해줘", "치워줘", "분류해줘" = 세 물체 모두 넣기
_ALL = re.compile(r'(다|전부|모두|세\s*개|셋\s*다|싹\s*다)\s*(다\s*)?(넣|정리|옮겨|분류|담아)')
_HOME = re.compile(r'(home|홈)\s*(으로|자세로|자세)?\s*(돌아|복귀|가|보내|돌려)|팔\s*(을\s*)?접어|(home|홈)\s*복귀|(home|홈)\s*자세\s*로?\s*[.!]*$', re.I)
_CALL = re.compile(r'^\s*(자비스|재비스|자비쓰|jarvis)\s*(야|아|님)?[\s,.!~]*', re.I)   # "자비스, 홈으로 가" 같은 부르는 말
_WHO = re.compile(r'이름이?\s*(뭐|무엇)|누구(야|니|세요|예요|에요)')
# 움직이지 않는 질문은 LLM(노트북까지 왕복 수 초)을 거치지 않고 바로 답한다
_LOOK = re.compile(r'(뭐|뭘|무엇|무슨\s*물체).{0,8}(있|보여|넣을|놓여)|작업대|다시\s*(봐|보기)')
_STATUS = re.compile(r'상태|뭐\s*하고\s*있|어디까지|진행\s*(상황|중)')
_ARM_STATES = {'ok': '팔은 토크가 켜져 있어 움직일 수 있습니다.',
               'off_at_home': '팔은 HOME 위치에 있고 토크가 꺼져 있습니다. 작업을 시키면 토크를 자동으로 켭니다.',
               'off_near_fold': '팔이 접힌 HOME 근처에 있고 토크가 꺼져 있습니다. 작업을 시키면 토크를 켜고 HOME에 맞춘 뒤 시작합니다.',
               'off_elsewhere': '토크가 꺼져 있고 팔이 HOME이 아닌 곳에 있습니다. "홈으로 돌아가"라고 하면 현재 자세에서 붙잡고 HOME으로 복귀합니다(팔이 걸리거나 처지지 않게 받쳐 주세요). 다른 작업은 HOME에서만 됩니다.',
               'mixed': '일부 관절만 토크가 켜져 있습니다.',
               'unknown': '서보 상태를 읽지 못했습니다(USB와 전원을 확인하세요).'}
_AGAIN = re.compile(r'^\s*(다시|한\s*번\s*더|재시도)\s*(해|해줘|해\s*줘|해봐|해\s*봐|하자|시도해|시도해줘|실행해|실행해줘)?\s*[.!]*\s*$')
_CLAIMS_ACTION = re.compile(r'(시작합니다|진행합니다|넣습니다|옮깁니다|정리합니다|돌아갑니다|실행합니다)')   # 도구 없이 이렇게 말하면 거짓 안내다
# "책상으로 이동해줘", "휴지통 앞으로 가", "밥먹는곳까지 가자" -> 그 장소 라벨로 주행. "홈으로 가"는 위의 _HOME(팔 복귀)이 먼저 잡는다.
_DRIVE = re.compile(r'^(?P<place>\S.*?)\s*(?:앞|쪽|근처)?\s*(?:으로|로|에|까지)\s*(?:이동|가|와|데려|가자|출발)')
_WHERE = re.compile(r'^(지금\s*)?(너\s*)?(어디야|어디니|어디에\s*있|어디\s*있|위치\s*(가\s*)?(어디|알려)|현재\s*위치)')
_NOT = re.compile(r'(넣|정리|옮기|접)지\s*마|말고|하지\s*마|\?$|까\s*[?.]?\s*$|나요\s*[?.]?\s*$')


def strip_call(text):
    """앞에 붙은 로봇 이름("자비스야,")을 뗀다."""
    match = _CALL.match(text)
    return text[match.end():] if match else text


def rule_command(text):
    """분명한 명령이면 ('stop'|'proposal'|'ask', 값), 아니면 None(LLM에 넘긴다). text는 strip_call을 거친 말."""
    if not text.strip():                                # 이름만 불렀다
        return 'ask', f'네, {ROBOT_NAME}입니다. 말씀하세요.'
    if _WHO.search(text):
        return 'ask', f'저는 {ROBOT_NAME}입니다. 물체를 칸에 넣거나 책상을 정리해 드려요.'
    if _STOP.match(text):
        return 'stop', None
    t = text.strip()
    if _NOT.search(t):
        return None
    lowered = t.lower()
    for a, b in _SYNONYMS:
        lowered = lowered.replace(a, b)
    if _HOME.search(lowered):
        return 'proposal', {'action': 'return_home'}
    if not _PUT.search(lowered):
        return None
    try:
        label, slot = parse_command(lowered)
    except ValueError as exc:
        if 'slot must be' in str(exc):
            return 'ask', '칸은 1, 2, 3번뿐입니다. 몇 번 칸에 넣을까요?'
        if 'could not tell' in str(exc) and (_ALL.search(lowered) or _TIDY.search(lowered)):   # 물체 이름 없이 "다 정리해줘", "책상 정리해줘"는 전부 넣기
            return 'proposal', {'action': 'pick_all'}
        return None                                   # 물체가 없거나 둘 이상: LLM이 되묻거나 나눠 처리한다
    if label == 'unknown' and slot is None:
        return 'ask', '새 물체를 몇 번 칸에 넣을까요? (1~3번)'
    proposal = {'action': 'pick_place', 'object': label}
    if slot is not None:
        proposal['slot'] = slot
    return 'proposal', normalize(proposal)


def normalize(proposal):
    """기본 칸을 굳이 적은 제안은 기본 칸 제안과 같게 만든다(메뉴 1~3번으로 간다)."""
    if proposal.get('action') == 'pick_place' and proposal.get('slot') == DEFAULT_SLOT.get(proposal.get('object')):
        proposal = {k: v for k, v in proposal.items() if k != 'slot'}
    return proposal


ORDER = ('white_box', 'red_cube', 'tape_measure')
WAIT = ' 바로 시작합니다.'      # 2026-10-21 사용자: 실행 여부를 묻지 말고 바로 실행(웹 화면이 제안을 받자마자 실행한다)


def read_desk(scene):
    """장면 → (넣을 수 있는 등록 물체, 보이지만 팔이 닿지 않는 등록 물체, 등록 안 된 물체 수). 순서는 메뉴 순서."""
    objects = scene.get('objects') or []
    present = [label for label in ORDER if any(o['label'] == label and o.get('pickable') for o in objects)]
    far = [label for label in ORDER if label not in present and any(o['label'] == label for o in objects)]
    unknown = sum(1 for o in objects if o['label'] == 'unknown')
    return present, far, unknown


def tray_sentence(in_tray):
    listed = ', '.join(f"{o['name'] if o['label'] != 'unknown' else '등록 안 된 물체'}({o['slot']}번 칸)" for o in in_tray)
    return f'트레이에는 {listed}가 이미 들어 있습니다(정리 대상에서 뺐습니다).'


def situation(arm):
    """대화 시작(그리고 작업이 끝날 때마다): 먼저 사진을 찍어 책상 상황을 알리고, 지금 쓸 수 있는 기능을 예시 문장과 함께 보여 준다.
    정리할 물체가 있으면 기본 행동인 책상 정리를 제안으로 준비한다('실행'이면 시작). 로봇은 움직이지 않는다.
    돌려주는 것: dict(reply, speech(소리로 읽을 짧은 문장), actions(기능 목록), proposal, scene)."""
    status, scene = arm.capture()
    always = ["HOME 복귀: '홈으로 돌아가'", "멈춤: '멈춰'", "다시 보기: '작업대에 뭐 있어?'"]
    if status != 200:
        why = scene.get('reason') or scene.get('reason_code') or status
        text = f'사진을 찍지 못했습니다({why}). 말이나 입력으로 명령해 주세요.'
        return dict(reply=text, speech=text, actions=always, proposal=None, scene=None)
    present, far, unknown = read_desk(scene)
    seen = [NAMES[label] for label in present + far] + ([f'등록 안 된 물체 {unknown}개'] if unknown else [])
    head = f"책상에서 {', '.join(seen)}를 찾았습니다." if seen else '책상에 정리할 물체가 없습니다.'
    if scene.get('in_tray'):
        head += ' ' + tray_sentence(scene['in_tray'])
    actions = []
    if present:
        actions.append(f"책상 정리: {', '.join(NAMES[label] for label in present)}를 모두 칸에 넣기 - '책상 정리해줘'")
        actions += [f"{NAMES[label]}만 넣기({DEFAULT_SLOT[label]}번 칸) - '{NAMES[label]} 넣어줘', 칸을 바꾸려면 '{NAMES[label]} 3번 칸에 넣어'" for label in present]
    if far:
        actions.append(f"{', '.join(NAMES[label] for label in far)}는 팔이 닿지 않습니다. 로봇 쪽으로 옮기면 넣을 수 있습니다.")
    if unknown:
        actions.append("새 물체 넣기 - '새 물체 3번 칸에 넣어'(칸을 꼭 말해 주세요)")
    actions += always
    if present:
        proposal = {'action': 'pick_all', 'objects': present}
        tail = "기본으로 책상 정리를 준비했습니다. '실행'이나 '정리해'라고 하면 바로 시작하고, 다른 일을 원하면 말해 주세요."
    else:
        proposal, tail = None, '무엇을 할까요?'
    return dict(reply=' '.join([head, tail]), speech=' '.join([head, tail]), actions=actions, proposal=proposal, scene=scene)


def tidy(arm):
    """책상 정리(기본 행동): 먼저 위 카메라로 찍어 책상에 있는 등록 물체 중 팔이 닿는 것만 넣는 제안을 만든다.
    돌려주는 것: (제안 또는 None, 답 문장, 찍은 장면 또는 None). 로봇은 움직이지 않는다."""
    status, scene = arm.capture()
    if status != 200:
        # 카메라를 못 쓰면(작업 중 등) 세 물체를 차례로 계획해 보고 책상에 없는 것은 건너뛴다
        why = scene.get('reason') or scene.get('reason_code') or status
        return {'action': 'pick_all'}, f'사진을 찍지 못했습니다({why}). 세 물체를 차례로 확인해 책상에 있는 것만 넣습니다.' + WAIT, None
    present, far, unknown = read_desk(scene)
    notes = []
    if far:
        notes.append(f"{', '.join(NAMES[label] for label in far)}는 보이지만 팔이 닿지 않는 위치라 뺐습니다.")
    if unknown:
        notes.append(f"등록 안 된 물체가 {unknown}개 있습니다. 넣으려면 '새 물체 3번 칸에 넣어'처럼 칸을 말해 주세요.")
    if scene.get('in_tray'):
        notes.append(tray_sentence(scene['in_tray']))
    if not present:
        return None, ' '.join(['책상에 정리할 물체가 없습니다.'] + notes), scene
    proposal = {'action': 'pick_all', 'objects': present}
    return proposal, proposal_reply(proposal, notes), scene


def proposal_reply(proposal, notes=()):
    if proposal['action'] == 'return_home':
        what = '팔을 HOME 자세로 돌려놓습니다.'
    elif proposal['action'] == 'drive':
        what = f"{proposal['label']}(으)로 이동합니다(도착하면 앞 벽에 맞춰 붙습니다)."
        return what + WAIT
    elif proposal['action'] == 'pick_all' and proposal.get('objects'):
        found = proposal['objects']
        what = (f"책상에서 {', '.join(NAMES[label] for label in found)}를 찾았습니다. {len(found)}개를 차례로 넣습니다"
                f"({', '.join(f'{NAMES[label]} {DEFAULT_SLOT[label]}번 칸' for label in found)}).")
        what = ' '.join([what, *notes])
    elif proposal['action'] == 'pick_all':
        what = '세 물체를 모두 기본 칸에 넣습니다(흰 상자 → 빨간 정육면체 → 줄자).'
    elif proposal['object'] == 'unknown':
        what = f"새 물체를 {proposal['slot']}번 칸에 넣습니다. 시작한 뒤 안내가 나오면 물체를 놓아 주세요."
    else:
        what = f"{NAMES[proposal['object']]}를 {proposal.get('slot') or DEFAULT_SLOT[proposal['object']]}번 칸에 넣습니다."
    return what + WAIT


def stop_reply(outcome):
    return '정지 명령을 보냈습니다.' if outcome.get('stopped') else '실행 중인 동작이 없습니다.'
IMMEDIATE = {'look_at_table', 'get_arm_status', 'stop_arm'}


class LlmUnavailable(Exception):
    """Ollama에 연결하지 못했다(꺼져 있음·주소 오류)."""


def validate(name, args):
    """도구 이름·인자를 허용 목록으로 다시 검사한다. 문제가 있으면 문장, 없으면 None."""
    if name not in MOTION | IMMEDIATE:
        return '알 수 없는 도구입니다.'
    if not isinstance(args, dict):
        return '인자는 객체여야 합니다.'
    allowed = {'sort_object': {'object_name', 'slot'}, 'move_to_place': {'place'}}.get(name, set())
    if set(args) - allowed:
        return '알 수 없는 항목이 있습니다.'
    if name == 'move_to_place' and (not isinstance(args.get('place'), str) or not args['place'].strip()):
        return 'place(장소 이름)가 필요합니다.'
    if name == 'sort_object':
        if args.get('object_name') not in OBJECTS:
            return 'object_name은 white_box, red_cube, tape_measure, unknown 중 하나입니다.'
        if 'slot' in args and (isinstance(args['slot'], bool) or args['slot'] not in (1, 2, 3)):
            return '칸은 1, 2, 3 중 하나입니다.'
        if args['object_name'] == 'unknown' and 'slot' not in args:
            return '새 물체(unknown)는 놓을 칸(1~3)이 필요합니다. 사용자에게 칸을 물어보세요.'
    return None


def to_proposal(name, args):
    """웹 화면의 [실행]이 그대로 /api/menu/N 으로 바꿔 부르는 형태(POST /robot_arm과 같다)."""
    if name == 'sort_all_objects':
        return {'action': 'pick_all'}
    if name == 'arm_return_home':
        return {'action': 'return_home'}
    if name == 'move_to_place':
        return {'action': 'drive', 'label': args['place'].strip()}
    proposal = {'action': 'pick_place', 'object': args['object_name']}
    if 'slot' in args:
        proposal['slot'] = args['slot']
    return proposal


class OllamaClient:
    """Ollama /api/chat 호출. client(messages, tools)가 모델의 message dict를 돌려준다."""

    def __init__(self, host=None, model=None, timeout=None):
        self.host = (host or os.environ.get('OLLAMA_HOST') or 'http://127.0.0.1:11434').rstrip('/')
        if '://' not in self.host:
            self.host = 'http://' + self.host
        self.model = model or MODEL
        self.timeout = float(timeout or os.environ.get('ARM_LLM_TIMEOUT') or 120)

    def __call__(self, messages, tools):
        body = json.dumps({'model': self.model, 'messages': messages, 'tools': tools, 'stream': False, 'think': False,
                           'keep_alive': '10m', 'options': {'temperature': 0}}).encode()
        request = urllib.request.Request(self.host + '/api/chat', data=body, headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                return json.loads(response.read())['message']
        except urllib.error.HTTPError as error:
            detail = error.read().decode(errors='replace')[:200]
            if error.code == 404:
                raise LlmUnavailable(f'모델 {self.model}이(가) 없습니다. ollama pull {self.model} 후 다시 시도하세요.') from error
            raise RuntimeError(f'Ollama 오류 {error.code}: {detail}') from error
        except (urllib.error.URLError, ConnectionError, TimeoutError, OSError) as error:
            raise LlmUnavailable(f'Ollama({self.host})에 연결하지 못했습니다: {error}') from error


def _result_for_model(result):
    """모델에는 사진 바이트를 빼고 알려 준다(문맥을 아끼려고)."""
    return {k: v for k, v in result.items() if k != 'image_jpeg_base64'}


def drive_proposal(text):
    """장소로 가라는 말이면 (제안 또는 None, 답), 아니면 None. 지도에 없는 장소면 제안 없이 장소 목록을 알려 준다."""
    match = _DRIVE.match(text)
    if not match or _PUT.search(text) or '칸' in match['place']:
        return None
    place = match['place'].strip()
    try:
        label = base_drive.find_label(place, base_drive.base_state()['labels'])
    except base_drive.BaseError as exc:
        return None, str(exc)
    proposal = {'action': 'drive', 'label': label['name']}
    return proposal, proposal_reply(proposal)


def where_answer():
    try:
        return base_drive.where_reply(base_drive.base_state())
    except base_drive.BaseError as exc:
        return str(exc)


def retry(arm):
    """"다시 해줘": 마지막 작업을 다시 제안한다. 책상 정리는 사진을 새로 찍어 지금 책상에 있는 것만. 돌려주는 것: (제안 또는 None, 답, 장면 또는 None)."""
    status = arm.status()
    if status['state'] == 'busy':
        return None, '지금 작업 중입니다. 끝난 뒤 다시 말해 주세요.', None
    last = arm.task(status['task_id']) if status['task_id'] else None
    if not last:
        return None, '다시 할 작업이 없습니다. 무엇을 할까요?', None
    params = last.get('params') or {}
    if last['skill'] == 'pick_all':
        return tidy(arm)
    if last['skill'] == 'return_home':
        proposal = {'action': 'return_home'}
    elif last['skill'] == 'drive' and params.get('label'):
        proposal = {'action': 'drive', 'label': params['label']}
    elif last['skill'] == 'pick_place' and params.get('object') in OBJECTS:
        proposal = {'action': 'pick_place', 'object': params['object']}
        if params.get('slot'):
            proposal['slot'] = params['slot']
    else:
        return None, '마지막 작업은 다시 할 수 없는 종류입니다. 무엇을 할지 말해 주세요.', None
    proposal = normalize(proposal)
    return proposal, proposal_reply(proposal), None


def status_reply(arm):
    status = arm.status()
    if status['state'] == 'busy':
        return f"{status['phase_label'] or status['skill']} 진행 중입니다({status['elapsed_s']}초)."
    try:
        state = arm.read_state()
    except Exception:
        state = 'unknown'
    text = '대기 중입니다. ' + _ARM_STATES.get(state, f'팔 상태: {state}.')
    if status['task_state'] == 'failed':
        text += ' 마지막 작업은 실패했습니다.'
    return text


def chat(history, text, arm, client=None):
    """history: [{'role': 'user'|'assistant', 'content': 문자열}] (웹 화면이 보관), text: 이번 말.
    돌려주는 것: {'reply': 문장, 'proposal': 실행 대기 명령 또는 None, 'executed': 바로 실행한 명령 결과 목록, 'scene': 찍은 장면 또는 None}."""
    text = strip_call(text)
    rule = rule_command(text)
    if rule is not None:
        kind, value = rule
        if kind == 'stop':
            outcome = arm.stop()
            return dict(reply=stop_reply(outcome), proposal=None, executed=[dict(command='stop', result=outcome)], scene=None, source='rule')
        if kind == 'ask':
            return dict(reply=value, proposal=None, executed=[], scene=None, source='rule')
        if value['action'] == 'pick_all':               # 책상 정리: 사진부터 찍고 있는 물체만
            proposal, reply, scene = tidy(arm)
            return dict(reply=reply, proposal=proposal, executed=[], scene=scene, source='rule')
        return dict(reply=proposal_reply(value), proposal=value, executed=[], scene=None, source='rule')
    if _AGAIN.match(text):
        proposal, reply, scene = retry(arm)
        return dict(reply=reply, proposal=proposal, executed=[], scene=scene, source='rule')
    if _WHERE.match(text):
        return dict(reply=where_answer(), proposal=None, executed=[], scene=None, source='rule')
    drive = drive_proposal(text)
    if drive is not None:
        return dict(reply=drive[1], proposal=drive[0], executed=[], scene=None, source='rule')
    if not _PUT.search(text) and not _HOME.search(text):
        if _LOOK.search(text):
            found = situation(arm)
            executed = [dict(command='look_at_table', result=dict(ok=True))] if found['scene'] else []
            return dict(reply=found['reply'], proposal=found['proposal'], executed=executed, scene=found['scene'], source='rule')
        if _STATUS.search(text):
            return dict(reply=status_reply(arm), proposal=None, executed=[], scene=None, source='rule')
    messages = [{'role': 'system', 'content': SYSTEM}]
    messages += [{'role': m['role'], 'content': str(m['content'])} for m in history[-20:]
                 if m.get('role') in ('user', 'assistant') and m.get('content')]
    messages.append({'role': 'user', 'content': text})
    client = client or OllamaClient()
    proposal, executed, scene, reply = None, [], None, ''
    for _ in range(MAX_TURNS):
        message = client(messages, TOOLS)
        reply = (message.get('content') or '').strip()
        calls = message.get('tool_calls') or []
        if not calls:
            break
        messages.append({'role': 'assistant', 'content': message.get('content') or '', 'tool_calls': calls})
        for call in calls:
            function = call.get('function') or {}
            name, args = function.get('name'), function.get('arguments') or {}
            if isinstance(args, str):               # 일부 모델은 인자를 JSON 문자열로 준다
                try:
                    args = json.loads(args)
                except ValueError:
                    args = None
            problem = validate(name, args)
            if problem:
                outcome = dict(ok=False, error=problem)
            elif name == 'get_arm_status':
                outcome = arm.status()
            elif name == 'stop_arm':
                outcome = arm.stop()
                executed.append(dict(command='stop', result=outcome))
            elif name == 'look_at_table':
                status, outcome = arm.capture()
                if status == 200:
                    scene = outcome
                    executed.append(dict(command='look_at_table', result=dict(ok=True)))
                outcome = _result_for_model(outcome)
            elif proposal is not None:
                outcome = dict(ok=False, error='움직이는 명령은 한 번에 하나만 제안할 수 있습니다.')
            else:
                proposal = normalize(to_proposal(name, args))
                outcome = dict(ok=True, note='화면이 바로 실행한다.')
            messages.append({'role': 'tool', 'tool_name': name or 'unknown', 'content': json.dumps(outcome, ensure_ascii=False, default=str)})
    if proposal is not None and proposal['action'] == 'drive':
        try:                                             # 지도에 있는 장소인지 먼저 확인한다
            proposal = {'action': 'drive', 'label': base_drive.find_label(proposal['label'], base_drive.base_state()['labels'])['name']}
            reply = proposal_reply(proposal)
        except base_drive.BaseError as exc:
            proposal, reply = None, str(exc)
    elif proposal is not None and proposal['action'] == 'pick_all':
        proposal, reply, taken = tidy(arm)               # 책상 정리: 사진부터 찍고 있는 물체만
        scene = taken or scene
    elif proposal is not None:
        reply = proposal_reply(proposal)                 # 제안 뒤 문장은 서버가 정한다("넣는 중" 같은 오해 방지)
    elif any(e['command'] == 'stop' for e in executed):
        reply = stop_reply(next(e['result'] for e in executed if e['command'] == 'stop'))
    elif not executed and _CLAIMS_ACTION.search(reply or ''):
        reply = '무엇을 할지 분명하지 않아 아무것도 실행하지 않았습니다. 예: "흰 상자 넣어줘", "책상 정리해줘", "홈으로 돌아가".'
    return dict(reply=reply or '무엇을 할까요? 예: "빨간 큐브 넣어줘", "작업대에 뭐 있어?"', proposal=proposal, executed=executed, scene=scene, source='llm')
