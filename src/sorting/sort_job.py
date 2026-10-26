"""명령(물체 이름, 선택적으로 칸 번호) → 집기 목표와 place 작업 파일 내용. 하드웨어 접근 없음.

칸은 명령으로 지정하면 그 칸, 없으면 기본 규칙(흰 상자 1, 빨간 정육면체 2, 줄자 3). 놓는 높이는 칸 바닥과 실제로
옮길 물체의 높이로 계산한다. 위치는 후보 계산이며 물리 검증되지 않았다.
"""
import math

from perception.object_targets import footprint_radius_mm
from tray.tray_detect import drop_z_mm
from tray.tray_sorting import choose_slot

# '집는 위치가 짧다'는 사용자 요청. 이전 검증 집기는 10mm였지만, 실물 2회 접촉 기록(보정 0mm: 손목 세로 오차 -27px=약 6mm 짧음/성공, +10mm: +28px=약 6.5mm 넘침/실패)의
# 사이값 약 5mm가 맞다. 방사 방향으로만 더하고 높이는 그대로 둔다. 두 번의 측정에서 나온 추정치다.
DEFAULT_PICK_RADIAL_OFFSET_MM = 0.  # 2026-10-20 카메라 재보정 후 0(이전 5: 옛 카메라 기준)
# 기울여 집으면(먼 물체) 컵이 같은 목표에서 더 멀리(약 +9~10mm, 손목 사진 이미지 오차) 닿는다: 10도 기울임 접촉 사진 3건(2026-10-16)
# 평균 +8mm, 수직 3건 평균 -3mm. 이미지 1mm가 로봇 약 0.8mm(위 5mm 보정 때의 +10mm 비교)이므로 기울기 1도당 약 -0.9mm, 보수적으로 -0.8mm 줄인다.
TILT_RADIAL_MM_PER_DEG = -0.8
# 사용자 요청(2026-10-15): 칸 안으로 너무 깊게 들어간다 → 놓는 높이를 2cm 올린다(물체는 해제 뒤 2cm 떨어진다).
DROP_RAISE_MM = 20.
# 사용자 요청(2026-10-15): 1번 칸으로 너무 멀리 간다 → 1번 칸 목표를 로봇 쪽(반경 방향 안쪽)으로 2cm 옮긴다.
SLOT_RADIAL_OFFSET_MM = {}   # 2026-10-21: 1번 칸 고정 -20mm 대신 place_plan이 기울기·물체 높이로 컵을 안쪽에 둔다(물체 높이 × sin 기울기)


def build_job(targets, tray, catalog, label, commanded_slot=None, *, acknowledge_unverified_suction=False,
              accept_uncertain_tray=False, pick_radial_offset_mm=DEFAULT_PICK_RADIAL_OFFSET_MM):
    if (isinstance(pick_radial_offset_mm, bool) or not isinstance(pick_radial_offset_mm, (int, float))
            or not math.isfinite(pick_radial_offset_mm) or abs(pick_radial_offset_mm) > 10.):
        raise ValueError('pick radial offset must be a finite number within -10..10 mm')
    slot_number = choose_slot(label, commanded_slot)
    matches = [t for t in targets if t['label'] == label and (label != 'unknown' or t.get('new_object'))]
    if not matches:
        raise ValueError(f'{label} not found in the current camera view')
    if len(matches) > 1:
        raise ValueError(f'more than one {label} in view; refusing to guess which one to move')
    target = matches[0]
    if not target['pickable']:
        raise ValueError(f'{label} is not pickable: ' + '; '.join(target['problems']))
    if label == 'unknown':
        from perception.unknown_objects import unknown_catalog
        catalog = unknown_catalog(catalog, target.get('object_geometry'))
        if pick_radial_offset_mm:
            raise ValueError('unknown은 관측한 흡착 지점에서만 집을 수 있습니다.')
    warnings = []
    if not target['suction_verified']:
        if not acknowledge_unverified_suction:
            raise ValueError(f'suction has not been verified for {label}; acknowledge it explicitly to continue')
        warnings += [p for p in target['problems'] if 'suction' in p or 'flat' in p]
    if not tray.get('found'):
        raise ValueError('tray not found: ' + str(tray.get('reason')))
    uncertainty = tray.get('position_uncertainty_mm')
    if uncertainty:
        if not accept_uncertain_tray:
            raise ValueError(f'tray slot positions are uncertain by up to about {uncertainty:.0f} mm ({tray.get("method")} method); '
                             'accept explicitly or move the tray so its walls are visible in depth')
        warnings.append(f'tray slot positions are uncertain by up to about {uncertainty:.0f} mm')
    slot = tray['slots'][slot_number]
    entry = catalog['objects'][label]
    held_height = entry['height_mm'] + (entry['height_uncertainty_mm'] if label == 'unknown' else 0.)
    drop = drop_z_mm(slot['floor_height_mm'], held_height, catalog['cup_contact_offset_mm']) + DROP_RAISE_MM
    obstacles = [dict(label=t['label'], x_mm=t['target_xy'][0] * 1000., y_mm=t['target_xy'][1] * 1000.,
                      height_mm=float((t.get('object_geometry') or catalog['objects'][t['label']])['height_mm']),
                      radius_mm=footprint_radius_mm(t.get('object_geometry') or catalog['objects'][t['label']])) for t in targets if t is not target]
    camera = [target['target_xy'][0] * 1000., target['target_xy'][1] * 1000., target['contact_z'] * 1000.]
    radius = math.hypot(camera[0], camera[1])
    tilt_deg = float(target.get('pick_tilt_deg') or 0.)
    tilt_radial = TILT_RADIAL_MM_PER_DEG * tilt_deg
    scale = (radius + float(pick_radial_offset_mm) + tilt_radial) / radius
    manual = [camera[0] * scale, camera[1] * scale, camera[2]]
    # 위치별 옆 방향 보정(config/pick_xy_correction.json, 빨간 정육면체 접촉 사진으로 측정, 2026-10-16)
    from perception import pick_correction
    if label == 'unknown':
        tangential = 0.
    else:
        manual[0], manual[1], tangential = pick_correction.apply(manual[0], manual[1])
    slot_x, slot_y = slot['x_mm'], slot['y_mm']
    shift = SLOT_RADIAL_OFFSET_MM.get(slot_number, 0.)
    if shift:
        slot_radius = math.hypot(slot_x, slot_y)
        slot_x, slot_y = slot_x * (slot_radius + shift) / slot_radius, slot_y * (slot_radius + shift) / slot_radius
    job = dict(label=label, slot=dict(x_mm=slot_x, y_mm=slot_y, drop_z_mm=drop),
               rim_height_mm=float(tray['rim_height_mm']), obstacles=obstacles)
    if label == 'unknown':
        job['object_geometry'] = target['object_geometry']
    if commanded_slot is not None:
        job['slot_number'] = slot_number
    return dict(job=job, slot_number=slot_number, slot_source='command' if commanded_slot is not None else 'default_rule',
                manual_target_mm=manual, camera_target_mm=camera, pick_tilt_deg=target.get('pick_tilt_deg') or 0., pick_radial_offset_mm=float(pick_radial_offset_mm), pick_tangential_correction_mm=round(tangential, 2), pick_tilt_radial_correction_mm=round(tilt_radial, 2),
                warnings=warnings, tray_method=tray.get('method'), tray_numbering=tray.get('numbering'),
                physical_accuracy_verified=False, usable_for_motion=False)


import re

_OBJECT_WORDS = (
    ('white_box', ('흰 상자', '하얀 상자', '흰상자', '하얀상자', 'white box')),
    ('red_cube', ('빨간 상자', '빨강 상자', '빨간상자', '빨간 정육면체', '빨강 정육면체', '정육면체', 'red cube', 'red box')),
    ('tape_measure', ('줄자', 'tape measure', 'tape')),
    ('unknown', ('unknown', '새 물체', '미등록 물체')),
)
_ORDINAL = {'첫': 1, '한': 1, '두': 2, '세': 3, '셋': 3}


def parse_command(text):
    """"빨간 상자 3번 칸에 넣어줘" → ('red_cube', 3). 칸이 없으면 None(기본 규칙). 애매하면 ValueError."""
    lowered = text.lower()
    from perception import object_registry
    found = object_registry.mentioned(lowered)         # 등록 물체 이름이 먼저(기본 물체 별칭과 겹치는 이름은 등록할 때 막는다)
    found += [label for label, words in _OBJECT_WORDS if any(w in lowered for w in words)]
    if len(found) != 1:
        raise ValueError('could not tell which object to move' if not found else f'more than one object in the command: {found}')
    numbers = {int(n) for n in re.findall(r'(\d+)\s*번', lowered)}
    numbers |= {int(n) for n in re.findall(r'slot\s*(\d+)', lowered)}
    numbers |= {_ORDINAL[w] for w in re.findall(r'(첫|두|세|셋)\s*(?:번째|째)', lowered)}
    if len(numbers) > 1:
        raise ValueError(f'more than one slot number in the command: {sorted(numbers)}')
    if numbers and not numbers <= {1, 2, 3}:
        raise ValueError(f'slot must be 1, 2 or 3, got {sorted(numbers)}')
    return found[0], (numbers.pop() if numbers else None)
