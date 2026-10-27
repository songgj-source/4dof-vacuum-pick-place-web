"""분류된 물체 → 로봇 집기 목표(x, y, 접촉 z). 하드웨어 접근 없음. 후보 계산이며 모션 허가가 아니다.

- x, y: 깊이로 찾은 윗면 중심 픽셀을, 집기에 성공한 근거가 있는 평면 투영 방식(BoxLocator)에 넣는다.
  윗면이 낮은 물체는 그만큼 낮은 평면에 투영한다.
- 접촉 z: 사용자가 자로 잰 물체 높이(config/object_catalog.json) + 컵 접촉 보정.
  깊이로 잰 높이는 ±3mm 오차가 있어 쓰지 않는다.
- 깊이 윗면 중심과 기존 코드의 윗면 중심 픽셀은 빨간 정육면체에서 일정하게 (+17, +5.5)px
  어긋나서(4개 프레임), 그만큼 빼서 쓴다. 다른 물체에도 같다고 가정한 값이다.
"""
import json
import math
from pathlib import Path

import numpy as np

from perception import object_registry
from perception.box_target import BoxLocator, TABLE_PLANE_Z
from perception.object_classify import classify_objects
from robot.robot_model import load_planar_model, reachable
from tray.tray_sorting import slot_for

DEPTH_MINUS_TOP_CENTER_PX = (17., 5.5)
REFERENCE_TOP_MM = 40.        # TABLE_PLANE_Z가 맞춰진 빨간 정육면체 윗면 높이
ENVELOPE_M = (.16, .30)
PICK_TILTS_DEG = (0., 5., 10., 15., 20.)
PICK_REACH_MARGIN_M = .008
LABELS = ('red_cube', 'white_box', 'tape_measure')     # 기본 물체(카탈로그에 꼭 있어야 한다). 등록 물체는 object_registry.labels()
MIN_CONTACT_Z_M = .010        # 사용자 지시(2026-10-15)로 30mm 하한을 없앴다. 컵이 책상에 닿는 것만 막는 최소선(pick_auto --min-contact-z-mm)


# 등록 안 된 물체를 장애물로 볼 범위(arm_api.describe_scene과 같은 기준): 이 밖의 덩어리는 트레이·설비·로봇 자신·케이블로 보고 뺀다
UNREGISTERED_MAX_PX = 150
UNREGISTERED_MAX_HEIGHT_MM = 100.
UNREGISTERED_RADIUS_M = (.12, .42)
UNREGISTERED_HEIGHT_MARGIN_MM = 3.     # 깊이로 잰 높이 오차(약 ±3mm)만큼 윗면을 높게 잡는다
UNREGISTERED_MIN_RADIUS_MM = 20.


def footprint_radius_mm(entry, default=40.):
    """물체 윗면 가로×세로(footprint_mm)의 대각선 절반 = 방향을 몰라도 안전한 수평 반경. 크기를 모르면 가정값."""
    size = entry.get('footprint_mm') if isinstance(entry, dict) else None
    if isinstance(entry, dict) and 'footprint_radius_mm' in entry:
        radius = entry['footprint_radius_mm']
        if isinstance(radius, bool) or not isinstance(radius, (int, float)) or not math.isfinite(radius) or radius <= 0:
            raise ValueError('object footprint radius must be finite and positive')
        return float(radius)
    if not size:
        return float(default)
    return math.hypot(*size) / 2.


def load_catalog(path):
    data = json.loads(Path(path).read_text())
    offset = data.get('cup_contact_offset_mm')
    if isinstance(offset, bool) or not isinstance(offset, (int, float)) or not math.isfinite(offset):
        raise ValueError('cup_contact_offset_mm must be a finite number')
    objects = data.get('objects') or {}
    objects.update(object_registry.catalog_entries(Path(path).resolve().parents[1]))     # 등록 물체(config/registered_objects.json)
    data['objects'] = objects
    for label in LABELS:
        entry = objects.get(label)
        height = entry.get('height_mm') if isinstance(entry, dict) else None
        if isinstance(height, bool) or not isinstance(height, (int, float)) or not math.isfinite(height) or height <= 0:
            raise ValueError(f'catalog needs a positive height_mm for {label}')
    press = data.get('extra_press_mm', 0)
    if isinstance(press, bool) or not isinstance(press, (int, float)) or not math.isfinite(press) or not 0 <= press <= 10:
        raise ValueError('extra_press_mm must be a finite number within 0..10')
    for label, entry in objects.items():
        own = entry.get('extra_press_mm') if isinstance(entry, dict) else None
        if own is not None and (isinstance(own, bool) or not isinstance(own, (int, float)) or not math.isfinite(own) or not 0 <= own <= 10):
            raise ValueError(f'{label}: extra_press_mm must be a finite number within 0..10')
    return data


def extra_press_mm(catalog, label):
    """집을 때 접촉 높이에서 더 누르는 양(mm). 물체 항목에 extra_press_mm가 있으면 그 값, 없으면 카탈로그 공통 값."""
    entry = catalog['objects'].get(label) or {}
    return float(entry.get('extra_press_mm', catalog.get('extra_press_mm', 0)))


def _project(locator, pixel, plane_z):
    ray = np.array([(pixel[0] - locator.cx) / locator.fx, (pixel[1] - locator.cy) / locator.fy, 1.0])
    direction = locator.R.T @ ray
    origin = -locator.R.T @ locator.t
    scale = (plane_z - origin[2]) / direction[2]
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError('invalid camera-plane intersection')
    return (origin + scale * direction)[:2]


def default_locator(root):
    """물체(집기 목표) 위치 계산용 카메라 변환. 2026-10-26 Pi 재보정 파일(camera_transform_objects.json)이 있으면 그것을 쓴다.
    트레이 칸 계산(tray_detect._locator)은 칸이 보정 범위 밖(30~36cm)이라 예전 camera_transform_candidate.json을 그대로 쓴다."""
    base = Path(root) / 'output/ik_resume_20261012'
    objects = base / 'multipose_review/camera_transform_objects.json'
    transform = objects if objects.exists() else base / 'multipose_review/camera_transform_candidate.json'
    return BoxLocator(transform, base / 'moved_box_target.json')


def top_center_xy(found, top_mm, locator):
    """classify_objects 결과 하나와 윗면 높이(mm) → 로봇 기준 윗면 중심 xy(m). locate_targets와 같은 투영."""
    pixel = np.array(found['top_center_px']) - np.array(DEPTH_MINUS_TOP_CENTER_PX)
    plane_z = TABLE_PLANE_Z - (REFERENCE_TOP_MM - float(top_mm)) / 1000.
    return _project(locator, pixel, plane_z) + locator.offset_xy


def footprint_axis(found, top_mm, locator, entry):
    """윗면 회전 사각형(top_corners_px)을 윗면 높이 평면에 투영해 책상 위 물체 방향(로봇 xy)을 잰다.
    등록 크기(footprint_mm)와 맞지 않거나 잴 수 없으면 None. 결과: slot_wall.object_axis(axis_xy, half_mm)."""
    from sorting.slot_wall import object_axis
    corners = found.get('top_corners_px')
    if not corners or not entry.get('footprint_mm'):
        return None
    plane_z = TABLE_PLANE_Z - (REFERENCE_TOP_MM - float(top_mm)) / 1000.
    try:
        xy = [(_project(locator, np.array(c) - np.array(DEPTH_MINUS_TOP_CENTER_PX), plane_z) + locator.offset_xy) * 1000. for c in corners]
    except ValueError:
        return None
    return object_axis([(float(p[0]), float(p[1])) for p in xy], entry['footprint_mm'])


def blob_geometry(item, locator, top_mm=None):
    """classify_objects 덩어리 하나 → 로봇 기준 윗면 중심 xy(m), 윗면 높이(mm), 사진 사각형으로 잰 가로·세로(mm),
    중심에서 사각형 모서리까지 반경(mm, 방향을 몰라도 안전한 쪽). 투영할 수 없으면 None."""
    top_mm = float(item.get('top_height_mm', item['height_mm'])) if top_mm is None else float(top_mm)
    try:
        xy = top_center_xy(item, top_mm, locator)
        plane_z = TABLE_PLANE_Z - (REFERENCE_TOP_MM - top_mm) / 1000.
        x, y, w, h = item['bbox_px']
        corners = [_project(locator, np.array(c) - np.array(DEPTH_MINUS_TOP_CENTER_PX), plane_z) + locator.offset_xy
                   for c in ((x, y), (x + w, y), (x, y + h), (x + w, y + h))]
    except ValueError:
        return None
    footprint = [float(np.linalg.norm(corners[1] - corners[0])) * 1000., float(np.linalg.norm(corners[2] - corners[0])) * 1000.]
    radius = max(math.hypot(*(c - xy)) for c in corners) * 1000.
    return dict(xy=[float(xy[0]), float(xy[1])], top_mm=top_mm, footprint_mm=footprint, radius_mm=radius)


def desk_blobs(found, locator, top_margin_mm=0.):
    """책상 위 물체로 볼 덩어리(이름과 상관없이): 트레이·설비 같은 큰 덩어리, 로봇팔 자신(높은 것),
    로봇 밑동 근처·작업 범위 밖(케이블 등)은 뺀다(arm_api.describe_scene과 같은 기준). (덩어리, blob_geometry) 목록."""
    out = []
    for item in found:
        if max(item['bbox_px'][2:]) > UNREGISTERED_MAX_PX:
            continue
        top_mm = float(item.get('top_height_mm', item['height_mm'])) + top_margin_mm
        if top_mm > UNREGISTERED_MAX_HEIGHT_MM:
            continue        # 로봇팔 자신
        geometry = blob_geometry(item, locator, top_mm)
        if geometry is None or not UNREGISTERED_RADIUS_M[0] <= math.hypot(*geometry['xy']) <= UNREGISTERED_RADIUS_M[1]:
            continue        # 로봇 밑동 근처·작업 범위 밖
        out.append((item, geometry))
    return out


def locate_unregistered(bgr, depth, root, locator=None, found=None):
    """책상 위 '등록 안 된 물체'를 장애물로(집지 않는다). 운반 경로 충돌 검사(place_plan)가 이 물체도 피하게 한다.
    윗면 높이는 덩어리 상위 10% 높이 + 오차, 반경은 덩어리 사각형 모서리까지(방향을 몰라도 안전한 쪽).
    돌려주는 것: [{label='unregistered', target_xy, object_geometry{height_mm, footprint_radius_mm}, pickable=False}]"""
    locator = locator or default_locator(root)
    found = found if found is not None else classify_objects(bgr, depth)
    return [dict(label='unregistered', name='등록 안 된 물체', target_xy=geometry['xy'],
                 object_geometry=dict(height_mm=round(geometry['top_mm'], 1),
                                      footprint_radius_mm=round(max(UNREGISTERED_MIN_RADIUS_MM, geometry['radius_mm']), 1)),
                 pickable=False, obstacle_only=True, problems=['not a registered object'])
            for item, geometry in desk_blobs([i for i in found if i['label'] == 'unknown'], locator, UNREGISTERED_HEIGHT_MARGIN_MM)]


def locate_targets(bgr, depth, root, catalog=None, model=None, locator=None, min_contact_z=MIN_CONTACT_Z_M):
    root = Path(root)
    base = root / 'output/ik_resume_20261012'
    catalog = catalog or load_catalog(root / 'config/object_catalog.json')
    model = model or load_planar_model(root)
    locator = locator or default_locator(root)
    results = []
    known = object_registry.labels(root)          # 기본 세 물체 + 등록 물체
    for found in classify_objects(bgr, depth):
        label = found['label']
        if label not in known:
            continue
        entry = catalog['objects'][label]
        top_mm = float(entry['height_mm'])
        pixel = np.array(found['top_center_px']) - np.array(DEPTH_MINUS_TOP_CENTER_PX)    # 기록용(projection_pixel)
        xy = top_center_xy(found, top_mm, locator)
        contact_z = (top_mm + float(catalog['cup_contact_offset_mm']) - extra_press_mm(catalog, label)) / 1000.
        radius = float(math.hypot(*xy))
        problems = []
        if not ENVELOPE_M[0] <= radius <= ENVELOPE_M[1]:
            problems.append(f'radius {radius:.3f} m outside the {ENVELOPE_M[0]}-{ENVELOPE_M[1]} m pick envelope')
        if contact_z < min_contact_z - 1e-9:
            problems.append(f'contact height {contact_z * 1000:.1f} mm is below the {min_contact_z * 1000:.0f} mm floor of the motion code; this object cannot be picked')
        inner = max(.12, radius - .040)
        x, y = float(xy[0]), float(xy[1])
        # 수직으로 닿지 않는 먼 물체는 손목을 조금 기울여 집는다(사용자 지시 2026-10-15). 닿는 가장 작은 기울기를 고른다.
        # 집기 보정(+5mm 바깥쪽)과 위치 오차를 위해 반경 8mm 여유를 두고 검사한다.
        pick_tilt = None
        first_fail = None
        for tilt in PICK_TILTS_DEG:
            pitch = -math.pi / 2 + math.radians(tilt)
            grow = (radius + PICK_REACH_MARGIN_M) / radius
            fails = [(name, point) for name, point in (('inner approach', (x * grow * inner / radius, y * grow * inner / radius, .120)),
                                                      ('approach 70 mm', (x * grow, y * grow, .070)), ('contact', (x * grow, y * grow, contact_z)))
                     if not reachable(model, point, pitch)]
            if first_fail is None and fails:
                first_fail = fails[0]
            if not fails:
                pick_tilt = tilt
                break
        if pick_tilt is None:
            problems.append(f'not reachable ({first_fail[0]}) even with {PICK_TILTS_DEG[-1]:g} deg tilt: {tuple(round(v, 3) for v in first_fail[1])}')
        pickable = not problems          # 기하(작업 거리·도달)만 본 값이다. 아래 권고는 별도다.
        if not entry.get('suction_verified'):
            problems.append('suction not verified for this object')
        if not entry.get('top_flat', True):
            problems.append('top surface is not flat; vacuum seal is uncertain and the depth centre may be off')
        results.append(dict(label=label, name=found['name'], slot=slot_for(label),
                            target_xy=[x, y], contact_z=contact_z, radius_m=radius,
                            footprint_axis=footprint_axis(found, top_mm, locator, entry),
                            pickable=pickable, problems=problems, pick_tilt_deg=pick_tilt,
                            suction_verified=bool(entry.get('suction_verified')),
                            top_center_px=found['top_center_px'], projection_pixel=[float(v) for v in pixel],
                            catalog_height_mm=top_mm, measured_height_mm=found['height_mm'],
                            usable_for_motion=False, physical_accuracy_verified=False))
    return results
