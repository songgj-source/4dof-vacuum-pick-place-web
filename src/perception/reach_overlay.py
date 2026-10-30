"""사진 위에 팔이 집을 수 있는 범위와 놓을 수 있는 범위를 그린다(2026-10-30 사용자 요청). 하드웨어 접근 없음.

- 집기: 로봇 밑동에서 PICK_MIN_MM~PICK_MAX_MM 띠(초록). 16cm 안쪽은 팔이 접혀 못 집고, 70mm 접근 높이에서 팔이 닿는 한계가 27.8cm다
  (2026-10-29 팔 모델로 잰 값: 정육면체 28.0, 줄자·흰 상자 27.8cm). 물체 위치 계산과 같은 카메라 변환(default_locator)으로 그린다.
- 놓기: 물체를 든 채 칸에 놓을 수 있는 반경 한계. 가장 짧은 물체(흰 상자) 37.5cm를 실선, 정육면체 39.2cm를 점선으로. 칸 위치는 트레이 계산이 쓰는
  변환(tray_detect._locator)으로 구한 값이라 칸과 맞게 같은 변환으로 그린다.
- 물체마다 로봇까지 거리(cm)를 닿으면 초록·안 닿으면 빨강으로, 칸마다 번호와 거리를 놓을 수 있으면 파랑·없으면 빨강으로 적는다.
글자는 OpenCV 기본 글꼴이라 숫자와 영문만 쓴다(설명은 화면 쪽 범례).
"""
import math

import cv2
import numpy as np

PICK_MIN_MM = 160.
PICK_MAX_MM = 278.
PLACE_SAFE_MM = 375.             # 모든 물체가 놓을 수 있는 반경(흰 상자 37.6cm)
PLACE_RED_CUBE_MM = 392.         # 정육면체만(39.2cm)
ANGLE_SPAN_DEG = 85
GREEN, BLUE, RED, WHITE, DARK = (80, 200, 90), (255, 170, 40), (60, 60, 230), (255, 255, 255), (20, 20, 20)
DEPTH_MINUS_TOP_CENTER_PX = (17., 5.5)       # object_targets와 같다(깊이 픽셀 - 이 값 = 색 픽셀)
LIMIT = 4000                                  # 카메라 가까이로 투영된 점이 int32를 넘지 않게


def to_pixel(locator, x_m, y_m, plane_z):
    """로봇 좌표(m)의 점 → 사진 픽셀. BoxLocator.camera_plane_xy의 역변환. 카메라 뒤쪽이면 None."""
    world = np.array([x_m - float(locator.offset_xy[0]), y_m - float(locator.offset_xy[1]), plane_z])
    cam = locator.R @ world + locator.t
    if cam[2] <= 1e-6:
        return None
    return float(locator.fx * cam[0] / cam[2] + locator.cx), float(locator.fy * cam[1] / cam[2] + locator.cy)


def _arc(locator, radius_m, plane_z, step=2.):
    points = []
    for deg in np.arange(-ANGLE_SPAN_DEG, ANGLE_SPAN_DEG + 1e-6, step):
        px = to_pixel(locator, radius_m * math.cos(math.radians(deg)), radius_m * math.sin(math.radians(deg)), plane_z)
        points.append(None if px is None else (int(max(-LIMIT, min(LIMIT, px[0]))), int(max(-LIMIT, min(LIMIT, px[1])))))
    return points


def _polyline(img, points, color, thickness=2, dashed=False):
    run = []
    for index, point in enumerate(points):
        if point is None:
            run = []
            continue
        if run and (not dashed or index % 4 < 2):
            cv2.line(img, run[-1], point, color, thickness, cv2.LINE_AA)
        run.append(point)


def _label(img, text, org, color):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, .5, DARK, 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1, cv2.LINE_AA)


def annotate(bgr, root, scene=None):
    """bgr 사진에 범위를 그린 복사본을 돌려준다. scene(describe_scene 결과)이 있으면 물체·칸 거리도 적는다. 실패하면 원본 그대로."""
    try:
        from pathlib import Path
        from perception.box_target import TABLE_PLANE_Z
        from perception.object_targets import default_locator
        from tray.tray_detect import _locator as tray_locator
        out = bgr.copy()
        pick_locator, place_locator = default_locator(Path(root)), tray_locator(Path(root))
        # 집기 띠(초록, 반투명)
        outer, inner = _arc(pick_locator, PICK_MAX_MM / 1000., TABLE_PLANE_Z), _arc(pick_locator, PICK_MIN_MM / 1000., TABLE_PLANE_Z)
        if all(p is not None for p in outer + inner):
            polygon = np.array(outer + inner[::-1], np.int32)
            shade = out.copy()
            cv2.fillPoly(shade, [polygon], GREEN)
            out = cv2.addWeighted(shade, .22, out, .78, 0)
        _polyline(out, outer, GREEN, 2)
        _polyline(out, inner, GREEN, 2)
        # 놓기 한계(파랑): 실선 = 모든 물체, 점선 = 정육면체만
        _polyline(out, _arc(place_locator, PLACE_SAFE_MM / 1000., TABLE_PLANE_Z), BLUE, 2)
        _polyline(out, _arc(place_locator, PLACE_RED_CUBE_MM / 1000., TABLE_PLANE_Z), BLUE, 2, dashed=True)
        height = out.shape[0]
        _label(out, f'PICK {PICK_MIN_MM / 10:.0f}-{PICK_MAX_MM / 10:.1f}cm', (8, 20), GREEN)
        _label(out, f'PLACE <={PLACE_SAFE_MM / 10:.1f}cm (cube {PLACE_RED_CUBE_MM / 10:.1f})', (8, 40), BLUE)
        if scene:
            for obj in scene.get('objects') or []:
                position, pixel = obj.get('position_mm'), obj.get('pixel')
                if not position or not pixel:
                    continue
                distance = math.hypot(*position) / 10.
                ok = obj.get('pickable') if obj['label'] != 'unknown' else obj.get('reachable', True)
                color = GREEN if ok else RED
                center = (int(pixel[0] - DEPTH_MINUS_TOP_CENTER_PX[0]), int(pixel[1] - DEPTH_MINUS_TOP_CENTER_PX[1]))
                cv2.circle(out, center, 4, color, -1, cv2.LINE_AA)
                _label(out, f'{distance:.1f}' + ('*' if abs(obj.get('pick_offset_mm') or 0.) >= .5 else ''), (center[0] + 6, center[1] - 6), color)
            for number, radius in sorted((scene.get('slot_radius_mm') or {}).items()):
                angle = (scene.get('slot_angle_rad') or {}).get(number, (scene.get('slot_angle_rad') or {}).get(str(number)))
                if angle is None:
                    continue
                px = to_pixel(place_locator, float(radius) * math.cos(angle) / 1000., float(radius) * math.sin(angle) / 1000., TABLE_PLANE_Z)
                if px is None:
                    continue
                color = BLUE if float(radius) <= PLACE_SAFE_MM else (0, 215, 255) if float(radius) <= PLACE_RED_CUBE_MM else RED
                center = (int(px[0]), int(px[1]))
                if 0 <= center[0] < out.shape[1] and 0 <= center[1] < height:
                    cv2.circle(out, center, 5, color, 2, cv2.LINE_AA)
                    _label(out, f'{number}:{float(radius) / 10:.1f}', (center[0] + 8, center[1] + 4), color)
        return out
    except Exception:
        return bgr
