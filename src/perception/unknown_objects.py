"""등록하지 않은 새 물체의 깊이 기반 후보. 장치 접근·물리 보정 승인 없음."""
import copy
import math

import cv2
import numpy as np

from perception.object_classify import _validate, table_height_mm, DEFAULT_INTRINSICS
from perception.object_targets import _project, default_locator, DEPTH_MINUS_TOP_CENTER_PX, ENVELOPE_M
from perception.box_target import BoxLocator
from robot.robot_frame import BARE_TABLE_PLANE_Z
from robot.robot_model import load_planar_model, reachable

MIN_HEIGHT_MM, MAX_HEIGHT_MM = 20., 45.
MAX_FOOTPRINT_MM = 120.
HEIGHT_UNCERTAINTY_MM = 3.
MIN_COUNTED_TOP_MM = 16.        # 새 물체로 세는 덩어리의 최소 윗면 높이(지원 물체는 20mm 이상, 깊이 오차 여유). 트레이 칸 바닥(약 11mm)은 세지 않는다
EXISTING_MAX_HEIGHT_MM = 120.   # 이보다 높게 찍힌 기준 장면의 것은 기존 물체 이동·제거 검사에서 뺀다(로봇팔·케이블)
MIN_PATCH_DIAMETER_MM = 20.  # 초기 지원 범위. 실제 컵 크기에 대한 물리 검증은 별도.
# 깊이로 잰 윗면 높이는 ±3mm 오차가 있어 가끔 컵이 닿기 전에 멈춰 흡착이 안 된다(사용자 2026-10-21).
# 그래서 측정한 윗면보다 더 내려가 흡착한 뒤 들어 올린다(스프링 컵이 눌리며 밀착). 10-20 성공은 보정 0에서였다.
# 2mm였던 것을 사용자 지시(2026-10-28)로 5mm로 늘렸다. 높이 20mm 물체의 접촉 높이가 15mm로 pick_auto 수동 목표 하한(15mm)에 딱 맞는다.
CONTACT_PRESS_MM = 5.


def validate_geometry(geometry):
    if not isinstance(geometry, dict) or geometry.get('source') != 'depth_table_plane':
        raise ValueError('unknown object geometry is required')
    height = geometry.get('height_mm')
    size = geometry.get('footprint_mm')
    uncertainty = geometry.get('height_uncertainty_mm')
    def finite(v):
        return not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v)
    if not finite(height) or not MIN_HEIGHT_MM <= height <= MAX_HEIGHT_MM:
        raise ValueError('unknown 높이는 20~45mm 범위여야 합니다.')
    if not isinstance(size, (list, tuple)) or len(size) != 2 or any(not finite(v) or not 20 <= v <= MAX_FOOTPRINT_MM for v in size):
        raise ValueError('unknown 가로·세로 추정값은 20~120mm 범위여야 합니다.')
    if not finite(uncertainty) or not 0 <= uncertainty <= HEIGHT_UNCERTAINTY_MM or geometry.get('top_flat') is not True:
        raise ValueError('unknown 윗면·높이를 신뢰할 수 없습니다.')
    radius = geometry.get('footprint_radius_mm')
    if not finite(radius) or not math.hypot(*size)/2 <= radius <= MAX_FOOTPRINT_MM * math.sqrt(2):
        raise ValueError('unknown 흡착 지점 기준 외곽 반경을 신뢰할 수 없습니다.')
    return copy.deepcopy(geometry)


def unknown_catalog(catalog, geometry):
    geometry = validate_geometry(geometry)
    result = copy.deepcopy(catalog)
    result['objects']['unknown'] = dict(geometry, suction_verified=False)
    # 기존 물체에서 얻은 눌림 보정은 임의 물체에 재사용하지 않는다.
    result['cup_contact_offset_mm'] = 0.
    result['extra_press_mm'] = 0.
    return result


def locate_unknowns(bgr, depth, baseline_depth, root, *, locator=None, model=None):
    """모든 전경의 크기를 추정하고 기준 장면에 없던 물체만 new_object로 표시."""
    _validate(bgr, depth)
    _validate(bgr, baseline_depth)
    if depth.dtype != np.uint16 or baseline_depth.dtype != np.uint16:
        raise ValueError('깊이는 uint16 밀리미터 프레임이어야 합니다.')
    good = (depth > 0) & (baseline_depth > 0)
    if good.mean() < .5:
        raise ValueError('깊이 유효 영역 부족: 카메라와 작업대를 확인하세요.')
    delta = baseline_depth.astype(float) - depth.astype(float)
    if np.median(np.abs(delta[good])) > 3.:
        raise ValueError('기준 장면의 배경이 바뀌었습니다. 다시 시작하세요.')
    height = table_height_mm(depth)
    # 후보 위치 선택에만 공간 중간값을 사용한다. 아래 평탄도·유효 깊이
    # 검사는 원래 높이값으로 수행해 실제 요철이나 깊이 누락은 숨기지 않는다.
    smooth_height = cv2.medianBlur(height.astype(np.float32), 5)
    baseline_height = table_height_mm(baseline_depth)
    # 기준 장면에서 EXISTING_MAX_HEIGHT_MM보다 솟은 것은 책상 물체가 아니라 로봇팔·케이블이다(지원 물체는 45mm 이하). 팔이 막 멈춘 뒤 찍은 기준에
    # 화면 아래 가장자리의 케이블(높이 약 300mm)이 찍혔다가 사라져 '기존 물체가 이동·제거'로 중단됐다(2026-10-28, 124px). 그런 것은 이 검사에서 뺀다.
    removed = good & (baseline_height > 8.) & (baseline_height < EXISTING_MAX_HEIGHT_MM) & (delta < -8.)
    # Astra의 경계·반사 잡음은 떨어진 픽셀로 나타난다. 전체 합계로
    # 판단하면 정지 장면도 이동으로 오인하므로 공간적으로 이어진 영역을 본다.
    coherent = cv2.morphologyEx(removed.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    _, _, removal_stats, _ = cv2.connectedComponentsWithStats(coherent)
    largest_removed = int(removal_stats[1:, cv2.CC_STAT_AREA].max()) if len(removal_stats) > 1 else 0
    if largest_removed > 100:
        raise ValueError('기준 장면의 기존 물체가 이동·제거됐습니다. 다시 시작하세요.')
    raised = ((height > 8.) & (depth > 0)).astype(np.uint8)
    raised[:60] = 0
    raised[:, 440:] = 0
    raised = cv2.morphologyEx(raised, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(raised)
    locator = locator or default_locator(root)
    model = load_planar_model(root) if model is None else model
    found = []
    for index in range(1, count):
        x, y, w, h, area = (int(v) for v in stats[index])
        if area < 600:
            continue
        mask = labels == index
        changed = mask & good & (delta > 8.)
        is_new = changed.sum() / area >= .7
        if max(w, h) > 150 and not is_new:
            continue  # 고정된 트레이·설비. 트레이 벽은 별도 경로 검사에 반영한다.
        values = height[mask & (depth > 0)]
        top_height = float(np.percentile(values, 90))
        smooth_top = float(np.percentile(smooth_height[mask & (depth > 0)], 90))
        flat = mask & (np.abs(smooth_height - smooth_top) <= 1.5) & (depth > 0)
        distance = cv2.distanceTransform(flat.astype(np.uint8), cv2.DIST_L2, 5)
        py, px = np.unravel_index(int(distance.argmax()), distance.shape)
        # 첫 후보가 실패해도 다른 흡착 면을 검사한다. 원본 깊이의
        # 20mm 원판 전체에 대한 3mm 편차 기준은 그대로 유지한다.
        typical_z = float(np.median(depth[mask & (depth > 0)]))
        needed = max(1, int(math.ceil(MIN_PATCH_DIAMETER_MM / 2 * DEFAULT_INTRINSICS['fx'] / max(typical_z, 1))))
        gy, gx = np.ogrid[-needed:needed+1, -needed:needed+1]
        kernel = ((gx*gx + gy*gy) <= needed*needed).astype(np.uint8)
        valid = mask & (depth > 0)
        low = cv2.erode(np.where(valid, height, 1.e6).astype(np.float32), kernel)
        high = cv2.dilate(np.where(valid, height, -1.e6).astype(np.float32), kernel)
        inside = cv2.distanceTransform(mask.astype(np.uint8), cv2.DIST_L2, 5) >= needed
        candidates = inside & valid & ((high-low) <= 5.) & (smooth_height >= smooth_top-3.)   # 후보 고르기만. 최종 판정은 아래 3mm
        cy, cx = np.nonzero(candidates)
        fy, fx = np.nonzero(mask)
        order = np.argsort((cx-fx.mean())**2 + (cy-fy.mean())**2, kind='stable')
        # 부적합한 경우도 측정값을 남겨 진단할 수 있도록 최초 후보를 마지막에 둔다.
        choices = [(int(cx[i]), int(cy[i])) for i in order] + [(int(px), int(py))]
        flat_ok = False
        for px, py in choices:
            z = float(depth[py, px])
            radius_px = max(1, int(math.ceil(MIN_PATCH_DIAMETER_MM / 2 * DEFAULT_INTRINSICS['fx'] / max(z, 1))))
            yy, xx = np.ogrid[:depth.shape[0], :depth.shape[1]]
            patch = (xx - px)**2 + (yy - py)**2 <= radius_px**2
            patch_values = height[patch & valid]
            flat_ok = ((patch & valid).sum() >= .95 * patch.sum()
                       and inside[py, px] and len(patch_values) > 0
                       # 최대-최소(ptp)는 Astra 픽셀 잡음 하나로 정해진다(10-20 저장 9회 모두 2.6~3.0mm로 기준에 걸침 → 재확인이 무작위 실패).
                       # 같은 3mm 기준을 5~95% 범위에 적용한다(평평한 윗면 1.7~2.1mm, 실제 요철·모서리는 여전히 걸린다).
                       and np.percentile(patch_values, 95) - np.percentile(patch_values, 5) <= 3.)
            if flat_ok:
                break
        top_height = float(np.median(patch_values)) if flat_ok else top_height
        plane = BARE_TABLE_PLANE_Z + top_height / 1000.
        pixel = np.array([px, py], float) - np.array(DEPTH_MINUS_TOP_CENTER_PX)
        xy = _project(locator, pixel, plane) + locator.offset_xy
        corners = [_project(locator, np.array(p, float) - DEPTH_MINUS_TOP_CENTER_PX, plane) for p in ((x, y), (x+w, y), (x, y+h), (x+w, y+h))]
        points = (np.array(corners) + locator.offset_xy) * 1000.
        footprint = [float(np.ptp(points[:, axis]) + 2 * HEIGHT_UNCERTAINTY_MM) for axis in (0, 1)]
        footprint_radius = max(math.hypot(*footprint)/2,
                               float(np.linalg.norm(points - xy*1000., axis=1).max()) + math.sqrt(2)*HEIGHT_UNCERTAINTY_MM)
        contour = max(cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0], key=cv2.contourArea)
        outline = np.array([(_project(locator, p[0].astype(float) - DEPTH_MINUS_TOP_CENTER_PX, outline_plane) + locator.offset_xy)*1000.
                            for p in contour for outline_plane in (plane-HEIGHT_UNCERTAINTY_MM/1000., plane+HEIGHT_UNCERTAINTY_MM/1000.)], np.float32)
        hull = cv2.convexHull(outline).reshape(-1, 2) - xy*1000.
        geometry = dict(source='depth_table_plane', height_mm=top_height, footprint_mm=footprint,
                        footprint_radius_mm=footprint_radius, height_uncertainty_mm=HEIGHT_UNCERTAINTY_MM,
                        footprint_polygon_mm=hull.tolist(), grasp_xy_mm=(xy*1000.).tolist(),
                        top_flat=bool(flat_ok), physical_validated=False)
        problems = []
        if is_new and (mask & (baseline_height > 8.)).sum() / area > .05:
            problems.append('기존 물체와 겹치거나 붙어 있습니다. 떨어뜨려 놓으세요.')
        if x <= 0 or y <= 60 or x+w >= 440 or y+h >= 480:
            problems.append('물체 전체가 작업 영역 안에 보이도록 놓으세요.')
        try:
            validate_geometry(geometry)
        except ValueError as exc:
            problems.append(str(exc))
        radius = float(np.linalg.norm(xy))
        if not ENVELOPE_M[0] <= radius <= ENVELOPE_M[1]:
            problems.append(f'radius {radius:.3f} m outside the {ENVELOPE_M[0]}-{ENVELOPE_M[1]} m pick envelope')
        if is_new and max(w, h) > 150:
            problems.append('unknown 크기가 초기 지원 범위를 넘습니다.')
        contact_z = (top_height - CONTACT_PRESS_MM) / 1000.
        if not all(reachable(model, point) for point in ((*xy, .120), (*xy, .070), (*xy, contact_z))):
            problems.append('target not reachable with a vertical suction cup')
        found.append(dict(label='unknown', name='unknown', new_object=bool(is_new), target_xy=list(map(float, xy)),
                          contact_z=contact_z, radius_m=radius, pickable=not problems, problems=problems,
                          object_geometry=geometry, suction_verified=False, pick_tilt_deg=0.,
                          top_center_px=[int(px), int(py)], bbox_px=[x, y, w, h],
                          measured_height_mm=top_height, usable_for_motion=False, physical_accuracy_verified=False))
    return found


DESK_FILL_MARGIN_PX = 6      # 지우는 범위: 덩어리 사각형보다 이만큼 넓게(덩어리 가장자리 픽셀까지)
DESK_FILL_MIN_HEIGHT_MM = 5.


def desk_baseline(bgr, depth, root, *, tray_slots=None, locator=None, which=None, of=None):
    """'바로 집기'용 가상 기준 깊이: 지금 책상에 있는 등록 안 된 물체 자리를 책상 높이로 지운 깊이.
    이 기준과 비교하면 그 물체가 '새로 놓인 물체'로 보이므로, 기존 unknown 검사(높이·윗면·흡착 면·칸 여유·움직이기 전 재확인)를
    그대로 쓴다. 등록 물체·트레이·로봇 자신·작업 범위 밖 덩어리는 그대로 둔다(그것들은 새 물체가 아니다).
    돌려주는 것: (기준 깊이 uint16, 지운 물체 목록 [{bbox_px, xy, top_mm}]). 트레이 칸 안 덩어리(칸막이 등)는 빼고 센다.
    which(1부터)를 주면 새 물체가 여러 개일 때 사진에서 왼쪽부터 which번째 하나만 지운다(나머지는 기준에 그대로 남아 새 물체가 아니다).
    of를 주면 보이는 개수가 of와 같을 때만 진행한다(안내한 번호와 실제 순서가 어긋난 채 집지 않게)."""
    from perception.object_classify import classify_objects
    from perception.object_targets import default_locator, desk_blobs
    from tray.occupancy import slot_containing
    _validate(bgr, depth)
    locator = locator or default_locator(root)
    found = [(item, geometry) for item, geometry in desk_blobs([i for i in classify_objects(bgr, depth) if i['label'] == 'unknown'], locator, min_top_mm=MIN_COUNTED_TOP_MM)
             if not tray_slots or slot_containing(geometry['xy'], tray_slots, radius_mm=70.) is None]
    found = sorted(found, key=lambda pair: pair[0]['bbox_px'][0])       # 왼쪽 → 오른쪽(사진 기준)
    if of is not None and len(found) != of:
        raise ValueError(f'unknown count changed: expected {of}, now {len(found)}')
    if which is not None:
        if not 1 <= which <= len(found):
            raise ValueError(f'unknown not found: 새 물체 {which}번이 없습니다(보이는 새 물체 {len(found)}개).')
        found = [found[which - 1]]
    height = table_height_mm(depth)
    baseline = depth.astype(np.int32)
    for item, _ in found:
        x, y, w, h = item['bbox_px']
        m = DESK_FILL_MARGIN_PX
        y0, y1, x0, x1 = max(0, y - m), min(depth.shape[0], y + h + m), max(0, x - m), min(depth.shape[1], x + w + m)
        region = (height[y0:y1, x0:x1] > DESK_FILL_MIN_HEIGHT_MM) & (depth[y0:y1, x0:x1] > 0)
        # 위에서 내려다보는 카메라라 솟은 높이만큼 깊이를 더하면 책상면에 가깝다(새 물체 판정은 8mm 차이만 본다)
        baseline[y0:y1, x0:x1][region] += np.round(height[y0:y1, x0:x1][region]).astype(np.int32)
    objects = [dict(bbox_px=item['bbox_px'], xy=geometry['xy'], top_mm=round(geometry['top_mm'], 1)) for item, geometry in found]
    return np.clip(baseline, 0, 65535).astype(np.uint16), objects


class StableUnknown:
    """중복 대상·위치·높이 변화가 있으면 연속 안정 표본을 다시 센다."""
    def __init__(self, samples=3):
        self.samples, self.previous, self.count = samples, None, 0

    def update(self, targets):
        new = [t for t in targets if t.get('new_object')]
        if len(new) != 1 or not new[0]['pickable']:
            self.previous, self.count = None, 0
            return False
        target = new[0]
        if self.previous is None:
            self.count = 1
        else:
            xy = math.dist(target['target_xy'], self.previous['target_xy'])
            dz = abs(target['measured_height_mm'] - self.previous['measured_height_mm'])
            size = max(abs(a-b) for a, b in zip(target['object_geometry']['footprint_mm'], self.previous['object_geometry']['footprint_mm']))
            self.count = self.count + 1 if xy <= .003 and dz <= 2. and size <= 3. else 1
        self.previous = copy.deepcopy(target)
        return self.count >= self.samples
