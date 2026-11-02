"""등록 안 된 새 물체(unknown)가 무엇인지 비전 LLM에게 물어 이름을 붙인다. 안내 문구에만 쓴다.

색 분류가 이름을 붙이지 못한 덩어리의 사진을 잘라 Ollama의 비전 모델(기본 qwen3.5:9b)에 보여 주고
한국어 명사 한두 단어를 받는다. 이 이름은 짐작이다. 집기 가능 여부·칸·높이 같은 동작 판단에는 쓰지 않는다(깊이 측정이 맡는다).
Ollama가 꺼져 있거나 오래 걸리면 None을 돌려 주고 호출한 쪽이 '등록 안 된 물체'로 말한다.
환경 변수: OLLAMA_HOST, ARM_LLM_MODEL, ARM_NAMER_TIMEOUT(초, 기본 25), ARM_NAMER=0이면 끈다.
"""
import base64
import hashlib
import json
import os
import re
import urllib.request

import cv2
import numpy as np

PAD_RATIO = .30         # 물체 둘레 여유: 큰 쪽 길이의 비율(깊이-색 정합 오차 15~20px도 덮는다)
PAD_MIN_PX = 24
CROP_MIN_SIDE = 320     # 작은 덩어리는 이만큼 키워서 보여 준다(글씨·무늬가 보이게)
CROP_MAX_SIDE = 512
CONTEXT_WIDTH = 640     # 책상 전체 사진: 주변 물체·트레이와 크기를 견주게 한다
MAX_NAME_CHARS = 12
MIN_CONFIDENCE = .25
CACHE_MAX = 16
GENERIC = {'물체', '물건', '사물', '오브젝트', '아이템', '모름', '알수없음', '알 수 없음', '미상', 'unknown', 'object', 'item'}
_cache = {}


def build_prompt(hints=None):
    """비전 모델에게 보내는 질문. 재어 둔 색·크기·높이를 알려 주고(깊이 측정이라 모델의 짐작보다 정확하다) 일상 이름을 JSON으로 받는다."""
    hints = hints or {}
    facts = []
    if hints.get('color'):
        facts.append(f"색 {hints['color']}")
    if hints.get('size_mm'):
        facts.append(f"가로세로 약 {hints['size_mm'][0] / 10:.1f}×{hints['size_mm'][1] / 10:.1f}cm")
    if hints.get('height_mm'):
        facts.append(f"높이 약 {hints['height_mm'] / 10:.1f}cm")
    measured = f"측정한 값: {', '.join(facts)}. 이 크기에 맞는 물건이어야 합니다.\n" if facts else ''
    return ('책상 위에 놓인 물체 하나가 무엇인지 맞혀 주세요.\n'
            '첫 번째 사진은 그 물체를 위에서 확대한 것이고, 두 번째 사진은 책상 전체에서 그 물체 위치를 초록 네모로 표시한 것입니다. '
            '옆에 놓인 줄자(약 6.5cm)·상자·칸이 나뉜 트레이와 크기를 견주어 보세요.\n'
            f'{measured}'
            '일상에서 부르는 한국어 이름 한두 단어로 답하세요. 글씨·로고·눈금·무늬가 보이면 단서로 쓰고, 단서가 없으면 모양으로 부르세요 '
            '(모서리가 반듯한 정육면체는 정육면체, 납작한 직육면체 포장은 상자, 둥글고 긴 것은 원통). '
            '이름은 사진에 실제로 보이는 것만 근거로 하고, 흔한 물건 이름을 짐작으로 끼워 맞추지 마세요. 색은 이름에 넣지 마세요(색은 따로 적습니다). '
            '확실하지 않으면 가장 가까운 것을 고르되 confidence를 낮게 적고, 정말 알 수 없을 때만 name을 "모름"으로 하세요.\n'
            '아래 JSON만 출력하세요: {"name": "이름", "confidence": 0부터 1 사이 숫자}')


def color_name(hue, sat, val):
    """측정한 색(OpenCV 색상 0~179, 채도·밝기 0~255)의 한국어 색 이름(관형형). AI가 아니라 측정값으로 정한다. 색이 없으면 None.
    기준: 2026-10-28 화면에서 잰 갈색 상자(H17 S133 V61)와 노란 줄자(H18 S215 V135). 주황~노랑 계열이 어두우면(V<110) 갈색이다."""
    if val is None or sat is None:
        return None
    if hue is None or sat < 50:                       # 무채색: 밝기로
        return '흰' if val >= 170 else '회색' if val >= 85 else '검은'
    if hue <= 6 or hue >= 160:
        return '분홍색' if sat < 110 and val >= 140 else '빨간'
    if hue < 35:
        return '갈색' if val < 110 else '주황색' if hue <= 14 else '노란'
    return '초록색' if hue < 86 else '청록색' if hue < 100 else '파란' if hue < 131 else '보라색'


COLOR_WORDS = {'노란': ('노란', '노랑', '노랗'), '빨간': ('빨간', '빨강', '빨갛'), '파란': ('파란', '파랑', '파랗'), '검은': ('검은', '검정', '까만'),
               '흰': ('흰', '하얀', '하양', '백색'), '갈색': ('갈색', '밤색'), '초록색': ('초록', '녹색'), '주황색': ('주황', '오렌지'),
               '보라색': ('보라',), '분홍색': ('분홍', '핑크'), '회색': ('회색', '그레이'), '청록색': ('청록',)}


def label(color, guess):
    """'갈색 박스'처럼 색과 이름을 합친다. 이름에 이미 같은 색이 들어 있으면('노랑 펜') 중복하지 않는다. 둘 다 없으면 None."""
    if guess and (not color or any(word in guess for word in COLOR_WORDS.get(color, (color,)))):
        return guess
    return ' '.join(part for part in (color, guess or ('물체' if color else None)) if part) or None


def enabled():
    return os.environ.get('ARM_NAMER', '1') != '0'


def _crop(bgr, bbox):
    x, y, w, h = (int(v) for v in bbox)
    height, width = bgr.shape[:2]
    pad = max(PAD_MIN_PX, int(PAD_RATIO * max(w, h)))
    x0, y0, x1, y1 = max(x - pad, 0), max(y - pad, 0), min(x + w + pad, width), min(y + h + pad, height)
    crop = bgr[y0:y1, x0:x1]
    side = min(crop.shape[:2])
    scale = CROP_MIN_SIDE / side if side < CROP_MIN_SIDE else 1.
    scale = min(scale, CROP_MAX_SIDE / max(crop.shape[:2]))
    return cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC) if abs(scale - 1.) > .01 else crop


def _context(bgr, bbox):
    """책상 전체 사진에 물체 위치를 초록 네모로 표시한 것."""
    scale = CONTEXT_WIDTH / bgr.shape[1] if bgr.shape[1] > CONTEXT_WIDTH else 1.
    image = cv2.resize(bgr, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale != 1. else bgr.copy()
    x, y, w, h = (int(round(v * scale)) for v in bbox)
    cv2.rectangle(image, (x - 3, y - 3), (x + w + 3, y + h + 3), (0, 220, 0), 2, cv2.LINE_AA)
    return image


def clean(text):
    """모델 답에서 이름만: 첫 줄, 괄호·따옴표·마침표 제거, 너무 길면 버린다."""
    line = (str(text or '').strip().splitlines() or [''])[0]
    line = re.sub(r'\(.*?\)|\[.*?\]|["\'`*.!?]', '', line).strip()
    return line if 0 < len(line) <= MAX_NAME_CHARS else None


def parse_answer(answer):
    """모델 답(JSON 또는 이름 한 줄)에서 이름을 뽑는다. 너무 막연한 이름('물체')이거나 확신이 낮으면 None."""
    text = str(answer or '').strip()
    name, confidence = None, None
    match = re.search(r'\{.*\}', text, re.S)
    if match:
        try:
            data = json.loads(match.group(0))
            name, confidence = clean(data.get('name')), data.get('confidence')
        except (ValueError, AttributeError):
            name = None
    if name is None and not match and '{' not in text and '}' not in text:
        name = clean(text)                  # JSON이 아니라 이름 한 줄로 온 경우
    if name is None or name.replace(' ', '') in GENERIC or name in GENERIC:
        return None
    if isinstance(confidence, (int, float)) and not isinstance(confidence, bool) and confidence < MIN_CONFIDENCE:
        return None
    return name


def hints_for(item, depth=None):
    """분류 결과 한 건에서 비전 모델에게 알려 줄 측정값: 색·가로세로(mm)·높이(mm). 깊이가 없으면 크기는 뺀다."""
    hints = {}
    if item.get('color_hsv'):
        hints['color'] = color_name(*item['color_hsv'])
    height = item.get('top_height_mm') or item.get('height_mm')
    if height:
        hints['height_mm'] = float(height)
    try:
        if depth is not None and item.get('bbox_px') and item.get('center_px'):
            from perception.object_classify import DEFAULT_INTRINSICS
            cx, cy = (int(v) for v in item['center_px'])
            z = float(depth[min(max(cy, 0), depth.shape[0] - 1), min(max(cx, 0), depth.shape[1] - 1)])
            if z > 0:
                w, h = item['bbox_px'][2], item['bbox_px'][3]
                hints['size_mm'] = (w * z / DEFAULT_INTRINSICS['fx'], h * z / DEFAULT_INTRINSICS['fy'])
    except Exception:
        pass
    return hints


def _ollama(images_b64, prompt, timeout):
    host = (os.environ.get('OLLAMA_HOST') or 'http://127.0.0.1:11434').rstrip('/')
    schema = dict(type='object', properties=dict(name=dict(type='string'), confidence=dict(type='number')), required=['name', 'confidence'])
    body = json.dumps(dict(model=os.environ.get('ARM_LLM_MODEL') or 'qwen3.5:9b', stream=False, think=False, format=schema,
                           options=dict(temperature=0),
                           messages=[dict(role='user', content=prompt, images=list(images_b64))])).encode()
    request = urllib.request.Request(host + '/api/chat', data=body, headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.load(response)['message']['content']


def name_object(bgr, bbox, client=None, hints=None):
    """덩어리(bbox_px)의 이름 추정. 실패하면 None. 확대한 물체 사진 + 책상 전체에서 위치를 표시한 사진 + 잰 크기·색을 함께 보낸다.
    같은 사진 조각은 저장해 둔 이름을 쓴다(웹 화면이 자주 다시 찍는다). client(images_b64, prompt, timeout)는 시험용으로 바꿀 수 있다."""
    if not enabled():
        return None
    try:
        crop = _crop(bgr, bbox)
        # 캐시 키: 작게 줄인 조각의 해시. 조명 잡음에 흔들리지 않게 거칠게 줄인다
        small = cv2.resize(crop, (8, 8), interpolation=cv2.INTER_AREA) // 32
        key = hashlib.sha1(np.ascontiguousarray(small).tobytes()).hexdigest()
        if key in _cache:
            return _cache[key]
        images = []
        for image in (crop, _context(bgr, bbox)):
            ok, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if not ok:
                return None
            images.append(base64.b64encode(buffer).decode())
        timeout = float(os.environ.get('ARM_NAMER_TIMEOUT') or 25)
        name = parse_answer((client or _ollama)(images, build_prompt(hints), timeout))
        if name:                                    # 실패(None)는 저장하지 않아 다음 사진에서 다시 시도한다
            if len(_cache) >= CACHE_MAX:
                _cache.pop(next(iter(_cache)))
            _cache[key] = name
        return name
    except Exception:
        return None
