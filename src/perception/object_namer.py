"""등록 안 된 새 물체(unknown)가 무엇인지 비전 LLM에게 물어 이름을 붙인다. 안내 문구에만 쓴다.

색 분류가 이름을 붙이지 못한 덩어리의 사진을 잘라 Ollama의 비전 모델(기본 qwen3.5:9b)에 보여 주고
한국어 명사 한두 단어를 받는다. 이 이름은 짐작이다. 집기 가능 여부·칸·높이 같은 동작 판단에는 쓰지 않는다(깊이 측정이 맡는다).
Ollama가 꺼져 있거나 오래 걸리면 None을 돌려 주고 호출한 쪽이 '등록 안 된 물체'로 말한다.
환경 변수: OLLAMA_HOST, ARM_LLM_MODEL, ARM_NAMER_TIMEOUT(초, 기본 20), ARM_NAMER=0이면 끈다.
"""
import base64
import hashlib
import json
import os
import re
import urllib.request

import cv2
import numpy as np

PROMPT = ('책상 위에 놓인 물체 하나를 위에서 찍은 사진입니다. 이 물체가 무엇인지 한국어 명사 한두 단어로만 답하세요. '
          '확실하지 않으면 가장 가까운 것을 고르세요. 설명이나 문장은 쓰지 마세요.')
PAD_PX = 30             # 깊이 덩어리 둘레 여유. 깊이-색 정합 오차(약 15~20px)가 있어 넉넉히 자른다
MAX_NAME_CHARS = 12
CACHE_MAX = 16
_cache = {}


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
    x0, y0, x1, y1 = max(x - PAD_PX, 0), max(y - PAD_PX, 0), min(x + w + PAD_PX, width), min(y + h + PAD_PX, height)
    crop = bgr[y0:y1, x0:x1]
    scale = max(1., 224. / max(crop.shape[:2]))     # 작은 덩어리는 키워서 보여 준다
    return cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC) if scale > 1 else crop


def clean(text):
    """모델 답에서 이름만: 첫 줄, 괄호·따옴표·마침표 제거, 너무 길면 버린다."""
    line = (str(text or '').strip().splitlines() or [''])[0]
    line = re.sub(r'\(.*?\)|\[.*?\]|["\'`*.!?]', '', line).strip()
    return line if 0 < len(line) <= MAX_NAME_CHARS else None


def _ollama(image_b64, timeout):
    host = (os.environ.get('OLLAMA_HOST') or 'http://127.0.0.1:11434').rstrip('/')
    body = json.dumps(dict(model=os.environ.get('ARM_LLM_MODEL') or 'qwen3.5:9b', stream=False, think=False,
                           options=dict(temperature=0),
                           messages=[dict(role='user', content=PROMPT, images=[image_b64])])).encode()
    request = urllib.request.Request(host + '/api/chat', data=body, headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.load(response)['message']['content']


def name_object(bgr, bbox, client=None):
    """덩어리(bbox_px)의 이름 추정. 실패하면 None. 같은 사진 조각은 저장해 둔 이름을 쓴다(웹 화면이 자주 다시 찍는다)."""
    if not enabled():
        return None
    try:
        crop = _crop(bgr, bbox)
        # 캐시 키: 작게 줄인 조각의 해시. 조명 잡음에 흔들리지 않게 거칠게 줄인다
        small = cv2.resize(crop, (8, 8), interpolation=cv2.INTER_AREA) // 32
        key = hashlib.sha1(np.ascontiguousarray(small).tobytes()).hexdigest()
        if key in _cache:
            return _cache[key]
        ok, buffer = cv2.imencode('.jpg', crop)
        if not ok:
            return None
        timeout = float(os.environ.get('ARM_NAMER_TIMEOUT') or 20)
        answer = (client or _ollama)(base64.b64encode(buffer).decode(), timeout)
        name = clean(answer)
        if name:                                    # 실패(None)는 저장하지 않아 다음 사진에서 다시 시도한다
            if len(_cache) >= CACHE_MAX:
                _cache.pop(next(iter(_cache)))
            _cache[key] = name
        return name
    except Exception:
        return None
