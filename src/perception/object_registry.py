"""등록 물체(사용자가 추가한 물체): 저장·불러오기, 색·크기·높이 특징으로 알아보기, 등록 검사. 하드웨어 접근 없음.

기본 세 물체(흰 상자·빨간 정육면체·줄자)는 config/object_catalog.json과 색 규칙(object_classify)이 그대로 맡는다.
등록 물체는 config/registered_objects.json에 따로 두고, 이 모듈이 분류·카탈로그·칸 규칙·메뉴 이름표에 끼워 넣는다.

특징(signature)은 등록 사진 한 장에서 잰 값이다: 윗면 높이, 화면 면적, 색(유채색이면 색상, 무채색이면 밝기).
한 조명·책상 조건에서 잰 값이라 조명이 크게 바뀌면 못 알아볼 수 있다(그때는 '등록 안 된 물체'로 남아 장애물로만 피한다).
흡착은 확인하지 않고 등록한다(사용자 지시 2026-10-26: 등록하면 바로 집는다). suction_verified=False로 기록한다.
"""
import json
import math
import os
import re
import tempfile
from datetime import datetime
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
REGISTRY_FILE = 'config/registered_objects.json'
PHOTO_DIR = 'output/registered_objects'
BUILTIN = ('red_cube', 'white_box', 'tape_measure')
BUILTIN_NAMES = {'white_box': '흰 상자', 'red_cube': '빨간 정육면체', 'tape_measure': '줄자'}
# 기존 이름·별칭(메뉴·채팅·NOMI가 이미 쓰는 말)과 겹치는 이름은 받지 않는다
RESERVED_NAMES = ('unknown', '새 물체', '새로운 물체', '모르는 물체', '등록 안 된 물체', '미등록 물체', '물체',
                  '흰 상자', '하얀 상자', '흰 박스', '박스', '상자', '빨간 정육면체', '빨간 상자', '빨간 큐브', '큐브', '정육면체',
                  '줄자', 'white_box', 'red_cube', 'tape_measure', 'tape', 'white box', 'red cube', 'tape measure')
# 이 말이 들어간 이름은 명령 해석(sort_job.parse_command)이 기본 물체로도 읽으므로 받지 않는다
CONTAINED_WORDS = ('흰 상자', '하얀 상자', '흰상자', '하얀상자', '빨간 상자', '빨강 상자', '빨간상자', '정육면체', '큐브', '줄자',
                   'tape', 'white box', 'red cube', 'red box', 'unknown', '새 물체', '미등록 물체')
MAX_REGISTERED = 3            # 책상 정리 한 번에 최대 6개(정리 시간·장애물 수 상한 8 안)
NAME_MAX = 20
HEIGHT_RANGE_MM = (20., 45.)  # unknown 자동 집기와 같은 범위(운반 높이 130mm·흡착 접근 범위 안)
FOOTPRINT_RANGE_MM = (20., 85.)   # 칸(92×100mm)에 들어가는 크기. 사진 사각형으로 재므로 비스듬히 놓으면 크게 잡힌다
# 알아보기 허용치
HEIGHT_TOL_MM = 6.            # 깊이 높이 오차(약 ±3mm)의 두 배
AREA_RATIO = (.6, 1.6)        # 화면 면적(놓는 방향·거리에 따라 달라진다)
HUE_TOL = 12.                 # OpenCV 색상(0~179)
VALUE_TOL = 40.               # 무채색 물체 밝기(0~255)
COLOR_KIND_SHARE = .3         # 덩어리에서 색이 남은 픽셀이 이 비율 이상이면 유채색 물체
BUILTIN_HEIGHT_GAP_MM = 9.    # 같은 색 종류의 기본 물체와 이만큼 높이가 달라야 구분한다(허용치 + 깊이 오차)
FLAT_TOP_SPREAD_MM = 4.       # 윗면 높이(상위 10%)와 중간 높이 차이가 이보다 작으면 윗면이 평평하다고 본다
# 기본 물체 색 종류(object_classify._color_masks와 같은 범위). 등록할 때 기본 물체와 헷갈리는지 본다
TAPE_LIKE_MIN_SATURATION = 160   # 줄자는 채도가 높다(중앙 204~228). 이보다 탁한 노랑·갈색 물체는 줄자와 헷갈리지 않는다(갈색 상자 119~139)
BUILTIN_COLORS = {'white_box': ('plain', None), 'red_cube': ('color', (160., 5.)), 'tape_measure': ('color', (10., 30.))}

_cache = {}


def _path(root=None):
    return Path(root or ROOT) / REGISTRY_FILE


def load(root=None):
    """등록 물체 목록(등록 순서). 파일이 없으면 []. 파일이 바뀔 때만 다시 읽는다."""
    path = _path(root)
    try:
        stamp = path.stat().st_mtime_ns
    except OSError:
        return []
    cached = _cache.get(path)
    if cached and cached[0] == stamp:
        return [dict(e) for e in cached[1]]
    data = json.loads(path.read_text())
    entries = data.get('objects') if isinstance(data, dict) else None
    if not isinstance(entries, list):
        raise ValueError(f'{REGISTRY_FILE}: objects must be a list')
    _cache[path] = (stamp, entries)
    return [dict(e) for e in entries]


def save(entries, root=None):
    """원자적으로 저장한다(쓰는 도중 다른 프로세스가 읽어도 반쯤 쓴 파일을 보지 않게)."""
    path = _path(root)
    path.parent.mkdir(parents=True, exist_ok=True)
    body = dict(source='물체 등록 기능(사진 한 장의 색·크기·높이). 흡착은 확인하지 않고 등록했다.', objects=entries)
    with tempfile.NamedTemporaryFile('w', dir=path.parent, delete=False, suffix='.tmp', encoding='utf-8') as out:
        json.dump(body, out, ensure_ascii=False, indent=2)
    os.replace(out.name, path)
    _cache.pop(path, None)


def labels(root=None):
    """정리 순서: 기본 세 물체 → 등록 물체(등록 순서)."""
    return list(BUILTIN) + [e['label'] for e in load(root)]


def names(root=None):
    return dict(BUILTIN_NAMES, **{e['label']: e['name'] for e in load(root)})


def find(label, root=None):
    return next((e for e in load(root) if e['label'] == label), None)


def catalog_entries(root=None):
    """object_catalog.json의 objects 형식으로(높이·가로세로·윗면·흡착). load_catalog가 합친다."""
    return {e['label']: dict(height_mm=float(e['height_mm']), footprint_mm=list(e['footprint_mm']), top_flat=bool(e['top_flat']),
                             suction_verified=False, registered=True) for e in load(root)}


def slots(root=None):
    return {e['label']: int(e['slot']) for e in load(root)}


def _key(text):
    return ''.join(str(text or '').split()).casefold()


def resolve_name(text, root=None):
    """등록 물체 이름(띄어쓰기·대소문자 무시) 또는 label → label. 없으면 None."""
    key = _key(text)
    return next((e['label'] for e in load(root) if key in (_key(e['name']), _key(e['label']))), None)


def mentioned(text, root=None):
    """문장에 들어 있는 등록 물체 label들(채팅·명령 해석용)."""
    key = _key(text)
    return [e['label'] for e in load(root) if _key(e['name']) and _key(e['name']) in key]


def sync_tables(names_table=None, slots_table=None, order=None, root=None):
    """모듈마다 가진 이름표·칸표·정리 순서에서 등록 물체 몫만 지금 등록 상태로 바꾼다(기본 물체 항목은 그대로 둔다).
    표를 새로 만들지 않고 그 자리에서 고쳐서, 이미 그 표를 참조하는 코드도 바뀐 내용을 본다."""
    entries = load(root)
    current = {e['label'] for e in entries}
    for table in (names_table, slots_table):
        if table is None:
            continue
        for label in [k for k in table if str(k).startswith('custom_') and k not in current]:
            del table[label]
    for e in entries:
        if names_table is not None:
            names_table[e['label']] = e['name']
        if slots_table is not None:
            slots_table[e['label']] = int(e['slot'])
    if order is not None:
        order[:] = [label for label in order if not str(label).startswith('custom_')] + [e['label'] for e in entries]


# ---- 특징과 알아보기 ----
def signature(hsv, chromatic, blob, top_mm, median_mm, area):
    """덩어리 하나의 특징. hsv·chromatic은 화면 전체, blob은 덩어리 픽셀(깊이 있는 곳)."""
    share = float(chromatic[blob].mean()) if blob.any() else 0.
    value = float(np.median(hsv[..., 2][blob])) if blob.any() else 0.
    sig = dict(kind='plain', hue=None, value=round(value, 1), color_share=round(share, 3), area_px=int(area),
               top_height_mm=round(float(top_mm), 1), top_spread_mm=round(float(top_mm) - float(median_mm), 1))
    if share >= COLOR_KIND_SHARE:
        angle = hsv[..., 0][blob & chromatic].astype(float) * (2 * math.pi / 180.)
        mean = math.atan2(float(np.sin(angle).mean()), float(np.cos(angle).mean())) % (2 * math.pi)
        sig.update(kind='color', hue=round(mean * 180. / (2 * math.pi), 1),
                   saturation=round(float(np.median(hsv[..., 1][blob & chromatic])), 1))
    return sig


def _hue_gap(a, b):
    d = abs(a - b) % 180.
    return min(d, 180. - d)


def close(sig, ref):
    """두 특징이 같은 물체로 볼 만큼 가까운가."""
    if sig['kind'] != ref['kind'] or abs(sig['top_height_mm'] - ref['top_height_mm']) > HEIGHT_TOL_MM:
        return False
    if not AREA_RATIO[0] <= sig['area_px'] / max(ref['area_px'], 1) <= AREA_RATIO[1]:
        return False
    if sig['kind'] == 'color':
        return _hue_gap(sig['hue'], ref['hue']) <= HUE_TOL
    return abs(sig['value'] - ref['value']) <= VALUE_TOL


def match(sig, entries):
    """특징에 맞는 등록 물체 label. 하나도 없거나 둘 이상 맞으면 None(추측하지 않는다)."""
    found = [e['label'] for e in entries if close(sig, e['signature'])]
    return found[0] if len(found) == 1 else None


def _like_builtin(sig, label, height_mm):
    """기본 물체와 색 종류가 같고 높이도 가까워 헷갈릴 수 있는가."""
    kind, hues = BUILTIN_COLORS[label]
    if sig['kind'] != kind or abs(sig['top_height_mm'] - height_mm) > BUILTIN_HEIGHT_GAP_MM:
        return False
    if kind == 'plain':
        return sig['value'] > 100 - VALUE_TOL       # 흰 상자 규칙(밝은 무채색)
    if label == 'tape_measure' and sig.get('saturation') is not None and sig['saturation'] < TAPE_LIKE_MIN_SATURATION:
        return False                                # 탁한 노랑·갈색은 줄자와 헷갈리지 않는다(밝기가 아니라 채도로 가른다)
    low, high = hues
    inside = low <= sig['hue'] or sig['hue'] <= high if low > high else low <= sig['hue'] <= high
    return inside or min(_hue_gap(sig['hue'], low), _hue_gap(sig['hue'], high)) <= HUE_TOL


# ---- 등록 ----
def check_name(name, root=None, entries=None):
    name = ' '.join(str(name or '').split())
    if not name:
        raise ValueError('물체 이름을 정해 주세요.')
    if len(name) > NAME_MAX:
        raise ValueError(f'이름은 {NAME_MAX}자 이하로 해 주세요.')
    if not re.search(r'[0-9A-Za-z가-힣]', name) or re.search(r'[,/\\<>"\'`{}]', name):
        raise ValueError('이름에는 한글·영문·숫자를 써 주세요(쉼표·따옴표·슬래시는 쓸 수 없습니다).')
    if _key(name) in {_key(n) for n in RESERVED_NAMES} or _key(name).startswith('custom_'):
        raise ValueError(f"'{name}'은(는) 이미 쓰는 이름입니다. 다른 이름을 정해 주세요.")
    word = next((w for w in CONTAINED_WORDS if _key(w) in _key(name)), None)
    if word:
        raise ValueError(f"이름에 기본 물체 이름('{word}')이 들어가면 명령을 헷갈립니다. 다른 이름을 정해 주세요.")
    entries = load(root) if entries is None else entries
    if any(_key(name) == _key(e['name']) for e in entries):
        raise ValueError(f"'{name}'은(는) 이미 등록된 이름입니다.")
    return name


def _check_number(value, low, high, what, unit='mm'):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{what}는 {low:g}~{high:g}{unit}여야 합니다(지금 {value}).')
    return float(value)


def candidate(bgr, depth, root=None, tray_slots=None, locator=None):
    """등록 사진에서 등록할 물체 하나: (분류 결과, 기하). 책상(트레이 밖·팔이 닿는 범위)에 물체가 정확히 하나여야 한다."""
    from perception.object_classify import classify_objects
    from perception.object_targets import default_locator, desk_blobs
    from tray.occupancy import slot_containing
    locator = locator or default_locator(root or ROOT)
    found = [(item, geometry) for item, geometry in desk_blobs(classify_objects(bgr, depth), locator)
             if not tray_slots or slot_containing(geometry['xy'], tray_slots, radius_mm=70.) is None]
    if len(found) != 1:
        raise ValueError(f'책상(트레이 밖)에 등록할 물체 하나만 두세요. 지금 {len(found)}개가 보입니다.'
                         + (' 다른 물체를 치워 주세요.' if found else ' 물체가 팔이 닿는 곳(로봇에서 12~42cm)에 있는지 확인하세요.'))
    return found[0]


def register(bgr, depth, name, slot, height_mm=None, root=None, tray_slots=None, locator=None, catalog=None):
    """사진 한 장으로 물체를 등록한다. height_mm(자로 잰 높이)가 없으면 깊이로 잰 윗면 높이를 쓴다. 등록한 항목을 돌려준다."""
    root = Path(root or ROOT)
    entries = load(root)
    if len(entries) >= MAX_REGISTERED:
        raise ValueError(f'물체는 {MAX_REGISTERED}개까지 등록할 수 있습니다. 먼저 하나를 삭제해 주세요.')
    name = check_name(name, root, entries)
    if isinstance(slot, bool) or slot not in (1, 2, 3):
        raise ValueError('칸은 1, 2, 3번 중 하나입니다.')
    item, geometry = candidate(bgr, depth, root, tray_slots, locator)
    sig = item['signature']
    if item['label'] in names(root) and item['label'] not in BUILTIN:
        raise ValueError(f"이 물체는 이미 등록된 '{names(root)[item['label']]}'와(과) 구분할 수 없습니다.")
    if catalog is None:
        catalog = json.loads((root / 'config/object_catalog.json').read_text())
    for label in BUILTIN:
        if _like_builtin(sig, label, float(catalog['objects'][label]['height_mm'])):
            raise ValueError(f"색과 높이가 기본 물체 '{BUILTIN_NAMES[label]}'와(과) 비슷해서 구분할 수 없습니다.")
    measured = float(sig['top_height_mm'])
    height = _check_number(measured if height_mm is None else height_mm, *HEIGHT_RANGE_MM, '높이')
    footprint = [round(v) for v in geometry['footprint_mm']]
    for side in footprint:
        _check_number(side, *FOOTPRINT_RANGE_MM, '가로·세로(사진으로 잰 값)')
    number = 1 + max([int(e['label'].split('_')[1]) for e in entries] + [0])
    entry = dict(label=f'custom_{number}', name=name, slot=int(slot), height_mm=round(height, 1),
                 height_source='depth' if height_mm is None else 'user', footprint_mm=footprint,
                 top_flat=sig['top_spread_mm'] <= FLAT_TOP_SPREAD_MM, suction_verified=False, signature=sig,
                 registered_at=datetime.now().astimezone().isoformat(timespec='seconds'))
    save(entries + [entry], root)
    try:                                            # 등록 사진을 남긴다(나중에 못 알아볼 때 비교용). 실패해도 등록은 유지
        import cv2
        folder = root / PHOTO_DIR
        folder.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(folder / f"{entry['label']}.png"), bgr)
    except Exception:
        pass
    return entry


def remove(label, root=None):
    """등록 물체 삭제. 기본 물체는 지울 수 없다. 지운 항목을 돌려준다."""
    if label in BUILTIN:
        raise ValueError('기본 물체(흰 상자·빨간 정육면체·줄자)는 삭제할 수 없습니다.')
    entries = load(root)
    target = next((e for e in entries if e['label'] == label or _key(e['name']) == _key(label)), None)
    if target is None:
        raise ValueError(f"'{label}'은(는) 등록된 물체가 아닙니다.")
    save([e for e in entries if e is not target], root)
    return target


def describe(entry):
    """사람에게 보여 줄 한 줄."""
    size = '×'.join(f'{v:g}' for v in entry['footprint_mm'])
    source = '자로 잼' if entry.get('height_source') == 'user' else '사진으로 잼'
    return (f"{entry['name']} → {entry['slot']}번 칸 (높이 {entry['height_mm']:g}mm {source}, 크기 약 {size}mm"
            + ('' if entry.get('top_flat') else ', 윗면 고르지 않음') + ')')
