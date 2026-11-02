"""사진 위에 팔이 집을 수 있는 범위와 놓을 수 있는 범위를 그린다(2026-10-30 사용자 요청). 하드웨어 접근 없음.

- 집기: 로봇 밑동에서 PICK_MIN_MM~PICK_MAX_MM 띠(초록). 16cm 안쪽은 팔이 접혀 못 집고, 70mm 접근 높이에서 팔이 닿는 한계가 27.8cm다
  (2026-10-29 팔 모델로 잰 값: 정육면체 28.0, 줄자·흰 상자 27.8cm). 물체 위치 계산과 같은 카메라 변환(default_locator)으로 그린다.
- 놓기: 물체를 든 채 칸에 놓을 수 있는 반경 한계. 가장 짧은 물체(흰 상자) 37.5cm를 실선, 정육면체 39.2cm를 점선으로. 칸 위치는 트레이 계산이 쓰는
  변환(tray_detect._locator)으로 구한 값이라 칸과 맞게 같은 변환으로 그린다.
- 물체마다 로봇까지 거리(cm)를 닿으면 초록·안 닿으면 빨강으로, 칸마다 번호와 거리를 놓을 수 있으면 파랑·없으면 빨강으로 적는다.
글자는 OpenCV 기본 글꼴이라 숫자와 영문만 쓴다(설명은 화면 쪽 범례).
2026-11-02 디자인 다듬기: 두꺼운 원색 선 대신 옅은 면 + 빛 번짐 + 가는 선, 거리는 둥근 알약 태그, 범례는 반투명 카드.
"""
import math

import cv2
import numpy as np

PICK_MIN_MM = 160.
PICK_MAX_MM = 278.
PLACE_SAFE_MM = 375.             # 모든 물체가 놓을 수 있는 반경(흰 상자 37.6cm)
PLACE_RED_CUBE_MM = 392.         # 정육면체만(39.2cm)
ANGLE_SPAN_DEG = 85
# 색(BGR): 민트 초록(집기), 하늘 파랑(놓기), 산호 빨강(못 닿음), 호박색(정육면체만 닿는 칸)
GREEN, BLUE, RED, AMBER = (128, 224, 120), (255, 190, 90), (96, 96, 255), (60, 200, 255)
WHITE, INK = (250, 250, 250), (28, 24, 22)
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


def _blend(img, draw, alpha):
    """draw(복사본)로 그린 것을 alpha만큼만 섞는다. 그리지 않은 곳은 그대로다."""
    layer = img.copy()
    draw(layer)
    return cv2.addWeighted(layer, alpha, img, 1. - alpha, 0)


def _polyline(img, points, color, thickness=1, dashed=False):
    run = []
    for index, point in enumerate(points):
        if point is None:
            run = []
            continue
        if run and (not dashed or index % 4 < 2):
            cv2.line(img, run[-1], point, color, thickness, cv2.LINE_AA)
        run.append(point)


def _glow_line(img, points, color, dashed=False, scale=1.):
    """빛 번짐(굵고 옅은 선) 위에 가는 선을 얹는다."""
    img = _blend(img, lambda layer: _polyline(layer, points, color, max(3, round(7 * scale)), dashed), .16)
    return _blend(img, lambda layer: _polyline(layer, points, color, max(1, round(1.6 * scale)), dashed), .95)


def _rounded(img, x0, y0, x1, y1, color, radius):
    cv2.rectangle(img, (x0 + radius, y0), (x1 - radius, y1), color, -1, cv2.LINE_AA)
    cv2.rectangle(img, (x0, y0 + radius), (x1, y1 - radius), color, -1, cv2.LINE_AA)
    for cx, cy in ((x0 + radius, y0 + radius), (x1 - radius, y0 + radius), (x0 + radius, y1 - radius), (x1 - radius, y1 - radius)):
        cv2.circle(img, (cx, cy), radius, color, -1, cv2.LINE_AA)


def _tag(img, text, anchor, color, scale=1., side='right'):
    """둥근 알약 태그: 어두운 반투명 바탕 + 왼쪽 색 점 + 흰 글자. anchor는 점 위치, 태그는 그 옆에 붙는다."""
    font, fs = cv2.FONT_HERSHEY_SIMPLEX, .44 * scale
    (tw, th), _ = cv2.getTextSize(text, font, fs, 1)
    pad, dot = round(7 * scale), round(4 * scale)
    w, h = tw + pad * 2 + dot * 2 + round(4 * scale), th + round(10 * scale)
    x0 = anchor[0] + round(8 * scale) if side == 'right' else anchor[0] - round(8 * scale) - w
    y0 = anchor[1] - h // 2
    h_img, w_img = img.shape[:2]
    x0, y0 = int(max(2, min(w_img - w - 2, x0))), int(max(2, min(h_img - h - 2, y0)))
    img = _blend(img, lambda layer: _rounded(layer, x0, y0, x0 + w, y0 + h, INK, h // 2), .72)
    cv2.circle(img, (x0 + pad + dot, y0 + h // 2), dot, color, -1, cv2.LINE_AA)
    cv2.putText(img, text, (x0 + pad + dot * 2 + round(4 * scale), y0 + h // 2 + th // 2), font, fs, WHITE, 1, cv2.LINE_AA)
    return img


def _legend(img, scale):
    """왼쪽 위 반투명 카드: 집기·놓기 범위."""
    font, fs = cv2.FONT_HERSHEY_SIMPLEX, .46 * scale
    rows = ((GREEN, f'PICK  {PICK_MIN_MM / 10:.0f} - {PICK_MAX_MM / 10:.1f} cm', False),
            (BLUE, f'PLACE  to {PLACE_SAFE_MM / 10:.1f} cm', False),
            (BLUE, f'cube  to {PLACE_RED_CUBE_MM / 10:.1f} cm', True))
    row_h, pad = round(20 * scale), round(10 * scale)
    width = max(cv2.getTextSize(text, font, fs, 1)[0][0] for _, text, _ in rows) + pad * 2 + round(34 * scale)
    x0, y0 = round(10 * scale), round(10 * scale)
    x1, y1 = x0 + width, y0 + row_h * len(rows) + pad
    img = _blend(img, lambda layer: _rounded(layer, x0, y0, x1, y1, INK, round(9 * scale)), .62)
    for index, (color, text, dashed) in enumerate(rows):
        y = y0 + pad // 2 + row_h * index + row_h // 2 + round(1 * scale)
        sx0, sx1 = x0 + pad, x0 + pad + round(22 * scale)
        if dashed:
            cv2.line(img, (sx0, y), (sx0 + round(8 * scale), y), color, 2, cv2.LINE_AA)
            cv2.line(img, (sx0 + round(13 * scale), y), (sx1, y), color, 2, cv2.LINE_AA)
        else:
            cv2.line(img, (sx0, y), (sx1, y), color, 3 if index == 0 else 2, cv2.LINE_AA)
        cv2.putText(img, text, (sx1 + round(9 * scale), y + round(5 * scale)), font, fs, WHITE, 1, cv2.LINE_AA)
    return img


def _arc_tag(img, points, text, color, scale, shape):
    """호가 사진 안쪽을 지나는 곳 한 군데에 거리 태그를 붙인다(호 가운데에서 먼저, 없으면 가장자리 쪽에서)."""
    height, width = shape[:2]
    margin = round(46 * scale)
    order = sorted(range(len(points)), key=lambda i: abs(i - len(points) // 2))
    for index in order:
        point = points[index]
        if point is not None and margin <= point[0] <= width - margin and margin * 2 <= point[1] <= height - margin:
            return _tag(img, text, point, color, scale, side='left' if point[0] > width // 2 else 'right')
    return img


def annotate(bgr, root, scene=None):
    """bgr 사진에 범위를 그린 복사본을 돌려준다. scene(describe_scene 결과)이 있으면 물체·칸 거리도 적는다. 실패하면 원본 그대로."""
    try:
        from pathlib import Path
        from perception.box_target import TABLE_PLANE_Z
        from perception.object_targets import default_locator
        from tray.tray_detect import _locator as tray_locator
        out = bgr.copy()
        height, width = out.shape[:2]
        scale = max(.8, width / 640.)
        pick_locator, place_locator = default_locator(Path(root)), tray_locator(Path(root))
        # 집기 띠: 옅은 면 + 가장자리 빛 번짐 선
        outer, inner = _arc(pick_locator, PICK_MAX_MM / 1000., TABLE_PLANE_Z), _arc(pick_locator, PICK_MIN_MM / 1000., TABLE_PLANE_Z)
        if all(p is not None for p in outer + inner):
            polygon = np.array(outer + inner[::-1], np.int32)
            out = _blend(out, lambda layer: cv2.fillPoly(layer, [polygon], GREEN), .22)
        out = _glow_line(out, outer, GREEN, scale=scale)
        out = _glow_line(out, inner, GREEN, scale=scale)
        # 놓기 한계: 실선 = 모든 물체, 점선 = 정육면체만
        safe = _arc(place_locator, PLACE_SAFE_MM / 1000., TABLE_PLANE_Z)
        cube = _arc(place_locator, PLACE_RED_CUBE_MM / 1000., TABLE_PLANE_Z)
        out = _glow_line(out, safe, BLUE, scale=scale)
        out = _glow_line(out, cube, BLUE, dashed=True, scale=scale)
        out = _arc_tag(out, outer, f'{PICK_MAX_MM / 10:.1f}', GREEN, scale, out.shape)
        out = _arc_tag(out, inner, f'{PICK_MIN_MM / 10:.0f}', GREEN, scale, out.shape)
        out = _legend(out, scale)
        if scene:
            for obj in scene.get('objects') or []:
                position, pixel = obj.get('position_mm'), obj.get('pixel')
                if not position or not pixel:
                    continue
                distance = math.hypot(*position) / 10.
                ok = obj.get('pickable') if obj['label'] != 'unknown' else obj.get('reachable', True)
                color = GREEN if ok else RED
                center = (int(pixel[0] - DEPTH_MINUS_TOP_CENTER_PX[0]), int(pixel[1] - DEPTH_MINUS_TOP_CENTER_PX[1]))
                cv2.circle(out, center, round(6 * scale), WHITE, 2, cv2.LINE_AA)
                cv2.circle(out, center, round(3 * scale), color, -1, cv2.LINE_AA)
                out = _tag(out, f'{distance:.1f}' + (' *' if abs(obj.get('pick_offset_mm') or 0.) >= .5 else ''), center, color, scale)
            for number, radius in sorted((scene.get('slot_radius_mm') or {}).items()):
                angle = (scene.get('slot_angle_rad') or {}).get(number, (scene.get('slot_angle_rad') or {}).get(str(number)))
                if angle is None:
                    continue
                px = to_pixel(place_locator, float(radius) * math.cos(angle) / 1000., float(radius) * math.sin(angle) / 1000., TABLE_PLANE_Z)
                if px is None:
                    continue
                color = BLUE if float(radius) <= PLACE_SAFE_MM else AMBER if float(radius) <= PLACE_RED_CUBE_MM else RED
                center = (int(px[0]), int(px[1]))
                if 0 <= center[0] < width and 0 <= center[1] < height:
                    cv2.circle(out, center, round(7 * scale), WHITE, 2, cv2.LINE_AA)
                    cv2.circle(out, center, round(3 * scale), color, -1, cv2.LINE_AA)
                    out = _tag(out, f'{number}  {float(radius) / 10:.1f}', center, color, scale)
        return out
    except Exception:
        return bgr
