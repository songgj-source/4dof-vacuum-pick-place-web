"""place 작업 파일(칸 위치·벽 높이·주변 물체)을 읽고 계획을 만든다. 하드웨어 접근 없음.

작업 파일은 서비스/계획 스크립트가 카메라로 구해 쓰고, pick_auto.py가 읽는다. 칸 번호는 파일에 있어도
물체 이름의 고정 규칙(빨간 정육면체 1, 흰 상자 2, 줄자 3)과 같을 때만 받아들인다.
"""
import json
import math
from pathlib import Path

from perception.object_targets import footprint_radius_mm
from sorting.place_plan import DEFAULT_MAX_TILT_DEG, min_carry_z, plan_place
from tray.tray_sorting import choose_slot

MAX_OBSTACLES = 8


def _finite(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'place job: {label} must be a finite number')
    return float(value)


def _inward(slot):
    """로봇 쪽 벽으로 당길 수 있는 거리(mm). 옛 작업 파일(값 없음)은 0 = 칸 중심."""
    value = _finite(slot.get('inward_max_mm', 0.), 'slot inward_max_mm')
    if not 0. <= value <= 40.:
        raise ValueError('place job: slot inward_max_mm must be within 0..40 mm')
    return value


def load_place_job(path):
    try:
        data = json.loads(Path(path).read_text())
    except (OSError, ValueError) as exc:
        raise ValueError(f'place job file unreadable: {exc}')
    if not isinstance(data, dict):
        raise ValueError('place job must be a JSON object')
    label = data.get('label')
    slot_number = choose_slot(label, data.get('slot_number'))
    geometry = None
    if label == 'unknown':
        from perception.unknown_objects import validate_geometry
        geometry = validate_geometry(data.get('object_geometry'))
    slot = data.get('slot')
    if not isinstance(slot, dict) or any(k not in slot for k in ('x_mm', 'y_mm', 'drop_z_mm')):
        raise ValueError('place job: slot needs x_mm, y_mm and drop_z_mm')
    rim = data.get('rim_height_mm')
    if rim is None:
        raise ValueError('place job: rim_height_mm is required (clearance over the tray rim must be checked)')
    rim = _finite(rim, 'rim_height_mm')
    if rim <= 0:
        raise ValueError('place job: rim_height_mm must be positive')
    raw = data.get('obstacles', [])
    if not isinstance(raw, list) or len(raw) > MAX_OBSTACLES:
        raise ValueError('place job: obstacles must be a short list')
    obstacles = []
    for item in raw:
        if not isinstance(item, dict) or any(k not in item for k in ('x_mm', 'y_mm', 'height_mm', 'radius_mm')):
            raise ValueError('place job: each obstacle needs x_mm, y_mm, height_mm and radius_mm')
        obstacles.append(dict(label=str(item.get('label', 'object')), x=_finite(item['x_mm'], 'obstacle x') / 1000.,
                              y=_finite(item['y_mm'], 'obstacle y') / 1000., height_mm=_finite(item['height_mm'], 'obstacle height'),
                              radius_mm=_finite(item['radius_mm'], 'obstacle radius')))
    result = dict(label=label, slot_number=slot_number,
                slot_source='command' if data.get('slot_number') is not None else 'default_rule',
                slot=dict(x=_finite(slot['x_mm'], 'slot x') / 1000., y=_finite(slot['y_mm'], 'slot y') / 1000.,
                          drop_z=_finite(slot['drop_z_mm'], 'slot drop z') / 1000.,
                          inward_max=_inward(slot) / 1000.),
                rim_height_mm=rim, obstacles=obstacles)
    if geometry is not None:
        result.update(object_geometry=geometry, unknown_observation=data.get('unknown_observation'),
                      slot_floor_height_mm=data.get('slot_floor_height_mm'), slot_axis_xy=data.get('slot_axis_xy'),
                      slot_center_mm=data.get('slot_center_mm'))
    return result


def build_plan(job, model, catalog, start_xyz, max_tilt_deg=DEFAULT_MAX_TILT_DEG, start_tilt_deg=0.):
    """작업 + 시작 자세(m) → place_plan 계획. 도달·벽 높이·주변 물체 검사에 실패하면 ValueError."""
    if job['label'] == 'unknown':
        from perception.unknown_objects import unknown_catalog
        catalog = unknown_catalog(catalog, job.get('object_geometry'))
    entry = catalog['objects'][job['label']]
    offset = float(catalog['cup_contact_offset_mm'])
    held = float(entry['height_mm'])
    if job['label'] == 'unknown':
        held += entry['height_uncertainty_mm']
        max_tilt_deg = 0.
    need = min_carry_z(held, job['rim_height_mm'], contact_offset_mm=offset)
    return plan_place(model, job['slot'], tuple(start_xyz), min_carry_z=need, held_height_mm=held,
                      obstacles=job['obstacles'], contact_offset_mm=offset, held_radius_mm=footprint_radius_mm(entry), max_tilt_deg=max_tilt_deg,
                      start_tilt_deg=start_tilt_deg)
