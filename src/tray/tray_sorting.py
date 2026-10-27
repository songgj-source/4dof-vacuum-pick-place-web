"""물체 이름 → 트레이 칸 번호 규칙과 칸 위치 설정. 하드웨어 접근 없음.

기본 규칙(사용자 지정): 칸 1 = 흰 상자, 칸 2 = 빨간 정육면체, 칸 3 = 줄자. 명령으로 칸을 지정하면 그 칸을 쓴다.
칸의 실제 위치는 트레이를 옮긴 뒤 측정해야 한다. 측정 전(config/tray_slots.json의
status가 'unmeasured')에는 어떤 칸도 위치를 돌려주지 않는다. 값을 추정해 채우지 않는다.
"""
import json
import math
from pathlib import Path
from types import MappingProxyType

# 기본 규칙(명령으로 칸을 지정하면 그 명령이 우선한다). 규칙표 자체는 읽기 전용이다.
# 2026-10-27 사용자: 트레이를 40cm까지 멀리 두려고 1·2번을 바꿨다. 가장 먼 1번 칸에 흰 상자는 39cm까지만 닿고 정육면체는 41cm까지 닿는다
SLOT_FOR_LABEL = MappingProxyType({'red_cube': 1, 'white_box': 2, 'tape_measure': 3})
SLOT_IDS = ('1', '2', '3')


def _rules():
    """기본 규칙 + 등록 물체(등록할 때 정한 칸)."""
    from perception import object_registry
    return dict(SLOT_FOR_LABEL, **object_registry.slots())
FIELDS = ('x_mm', 'y_mm', 'drop_z_mm')


def slot_for(label):
    rules = _rules()
    if label not in rules:
        raise ValueError(f'no tray slot for label {label!r}')
    return rules[label]


def choose_slot(label, commanded=None):
    """칸 번호: 명령으로 칸을 지정하면 그대로, 없으면 기본 규칙(빨간 정육면체 1, 흰 상자 2, 줄자 3).

    이름을 모르는 물체는 높이를 몰라 벽 위로 옮길 수 있는지·놓는 높이를 계산할 수 없어 거부한다.
    """
    rules = _rules()
    if label not in rules and label != 'unknown':
        raise ValueError(f'unknown object {label!r}: only white_box, red_cube, tape_measure and registered objects can be placed')
    if commanded is None:
        return 3 if label == 'unknown' else rules[label]
    if isinstance(commanded, bool) or not isinstance(commanded, int) or commanded not in (1, 2, 3):
        raise ValueError(f'slot must be 1, 2 or 3, got {commanded!r}')
    return commanded


def plan_sorting(objects):
    """분류 결과 목록 → 칸 배정. 이름을 모르는 물체는 건너뛰고 이유를 남긴다."""
    assignments, skipped, seen = [], [], set()
    rules = _rules()
    for o in objects:
        label = o.get('label')
        if label not in rules:
            skipped.append(dict(label=label, reason=f'label {label!r} has no tray slot'))
            continue
        if label in seen:
            raise ValueError(f'two objects share label {label!r}; refusing to guess which to move first')
        seen.add(label)
        assignments.append(dict(label=label, slot=rules[label], center_px=o.get('center_px'),
                                height_mm=o.get('height_mm')))
    return dict(assignments=assignments, skipped=skipped)


def load_slots(path):
    data = json.loads(Path(path).read_text())
    if data.get('status') not in ('measured', 'unmeasured'):
        raise ValueError("tray slots status must be 'measured' or 'unmeasured'")
    slots = data.get('slots')
    if not isinstance(slots, dict) or set(slots) - set(SLOT_IDS):
        raise ValueError('tray slots must be a dict with keys 1, 2, 3 only')
    rim = data.get('rim_height_mm')
    if rim is not None and (isinstance(rim, bool) or not isinstance(rim, (int, float))
                            or not math.isfinite(rim) or rim <= 0):
        raise ValueError('tray rim_height_mm must be a positive finite number')
    return dict(status=data['status'], slots=slots, rim_height_mm=None if rim is None else float(rim))


def require_slot(config, number):
    """칸 위치(m)를 돌려준다. 미측정이거나 값이 비정상이면 거부한다."""
    if config['status'] != 'measured':
        raise ValueError(f'tray slot {number} position is not measured')
    entry = config['slots'].get(str(number))
    if not isinstance(entry, dict):
        raise ValueError(f'tray slot {number} position is not measured')
    values = []
    for field in FIELDS:
        value = entry.get(field)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'tray slot {number} {field} is not a finite number')
        values.append(float(value))
    x, y, z = (v / 1000. for v in values)
    return dict(x=x, y=y, drop_z=z)
