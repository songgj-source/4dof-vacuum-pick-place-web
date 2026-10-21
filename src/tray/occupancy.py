"""물체가 이미 트레이 칸 안에 있는지 판단한다. 하드웨어 접근 없음.

칸에 넣은 물체를 '책상 위 물체'로 다시 보고 꺼내려 하지 않게 한다(2026-10-21: 3번 칸에 넣은 줄자를 상황 설명이 책상 위 줄자로 알렸다).
칸 중심(로봇 기준 mm)에서 IN_SLOT_MM 안에 물체 윗면 중심이 있으면 그 칸에 들어 있다고 본다. 칸은 약 92×100mm라
가장자리(약 46~50mm)보다 조금 넓게 잡는다. 트레이 바로 옆 책상에 놓인 물체는 칸 중심에서 60mm 넘게 떨어진다.
"""
import math

IN_SLOT_MM = 55.


def slot_containing(xy_m, slots, radius_mm=IN_SLOT_MM):
    """xy_m(로봇 기준 m)이 들어 있는 칸 번호, 없으면 None. slots: {번호: {'x_mm', 'y_mm', ...}} (detect_tray 결과)."""
    if not slots or xy_m is None:
        return None
    x, y = float(xy_m[0]) * 1000., float(xy_m[1]) * 1000.
    best = min(slots.items(), key=lambda kv: math.hypot(kv[1]['x_mm'] - x, kv[1]['y_mm'] - y))
    return int(best[0]) if math.hypot(best[1]['x_mm'] - x, best[1]['y_mm'] - y) <= radius_mm else None


def outside_tray(targets, slots):
    """locate_targets 결과에서 트레이 칸 안에 있는 것을 뺀다(계획이 칸 안 물체를 다시 집지 않게)."""
    return [t for t in targets if slot_containing(t['target_xy'], slots) is None]


# ---- 칸마다 직접 확인: 칸 바닥보다 솟은 높이(깊이)로 무엇이 들어 있는지 ----
# 칸 안 물체는 트레이 벽·칸막이와 한 덩어리로 잡히거나(빨간 정육면체), 칸막이가 '등록 안 된 물체'로 잡혀(2026-10-21)
# 물체 인식 결과만으로는 칸을 틀렸다. 물체 높이가 서로 달라(흰 상자 24, 줄자 30, 빨간 정육면체 40mm) 솟은 높이로 가른다.
# 칸 중심 색은 조명·위치에 따라 흔들려서(빨강 0.08~0.66) 분명할 때(COLOR_SURE 이상)만 따른다.
OCCUPIED_MM = 12.        # 칸 바닥보다 이만큼 솟으면 물체가 있다
HEIGHT_TOLERANCE_MM = 6.
COLOR_SURE = .30
WINDOW_PX = 12


def _fractions(hsv_patch):
    h, s, v = hsv_patch[:, 0], hsv_patch[:, 1], hsv_patch[:, 2]
    return dict(red_cube=float((((h < 10) | (h > 170)) & (s > 110) & (v > 60)).mean()),
                tape_measure=float(((h >= 15) & (h <= 38) & (s > 100) & (v > 100)).mean()),
                white_box=float(((s < 50) & (v > 160)).mean()))


def slot_contents(bgr, height, slots, color_pixel, depth_offset_px, heights_mm):
    """칸마다 [{'slot', 'label'(white_box·red_cube·tape_measure 또는 'unknown'), 'rise_mm'}] (물체가 있는 칸만).
    height: 책상 대비 높이 영상(mm, 깊이 픽셀). color_pixel(xy_m, height_mm) → 색 영상 (u, v).
    depth_offset_px: 색 픽셀 → 깊이 픽셀 차이. heights_mm: {라벨: 물체 높이}."""
    import cv2
    import numpy as np
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    rows, cols = height.shape[:2]
    found = []
    for number, slot in sorted(slots.items()):
        xy = (slot['x_mm'] / 1000., slot['y_mm'] / 1000.)
        floor = float(slot.get('floor_height_mm') or 0.)
        at = color_pixel(xy, floor + 20.)
        if at is None:
            continue
        du, dv = int(round(at[0] + depth_offset_px[0])), int(round(at[1] + depth_offset_px[1]))
        if not (WINDOW_PX <= du < cols - WINDOW_PX and WINDOW_PX <= dv < rows - WINDOW_PX):
            continue
        top = float(np.percentile(height[dv - WINDOW_PX:dv + WINDOW_PX + 1, du - WINDOW_PX:du + WINDOW_PX + 1], 80))
        rise = top - floor
        if rise < OCCUPIED_MM:
            continue
        label = min(heights_mm, key=lambda k: abs(heights_mm[k] - rise))
        if abs(heights_mm[label] - rise) > HEIGHT_TOLERANCE_MM:
            label = 'unknown'
        top_at = color_pixel(xy, top)
        if top_at is not None:
            cu, cv_ = int(round(top_at[0])), int(round(top_at[1]))
            patch = hsv[max(0, cv_ - WINDOW_PX):cv_ + WINDOW_PX + 1, max(0, cu - WINDOW_PX):cu + WINDOW_PX + 1].reshape(-1, 3)
            if len(patch):
                colors = _fractions(patch)
                best = max(colors, key=colors.get)
                if colors[best] >= COLOR_SURE:
                    label = best
        found.append(dict(slot=int(number), label=label, rise_mm=round(rise, 1)))
    return found
