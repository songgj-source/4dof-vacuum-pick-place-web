"""HOME 카메라 이미지에서 빨간 상자를 찾아 로봇 base 좌표 집기 목표를 계산한다.

하드웨어 접근 없음. 기존 수동 절차(상자 윗면 폴리곤 선택 → 카메라 광선과 평면
교점 → 성공한 집기 위치로 보정한 오프셋)를 자동화한 것이며, 정확도는 그 절차와
같다(카메라 교차검증 약 11 mm). 물리 검증된 값이 아니라 후보 계산이다.
"""
import json
import math
from pathlib import Path

import cv2
import numpy as np
from robot.robot_frame import BARE_TABLE_PLANE_Z, BASE_TABLE_PLANE_Z

# 상자 색: 색상이 0 근처(어두운 빨강)이고 채도가 높다. 골판지 트레이(H≈10)는 제외.
RED_LOW = ((0, 90, 20), (4, 255, 255))
RED_HIGH = ((170, 90, 20), (179, 255, 255))
MIN_AREA_PX = 800
MAX_AREA_PX = 6000
# 두 장면의 수동 윗면 폴리곤에서 구한 윗면 중심 위치(검출 bbox 대비 비율).
TOP_CENTER_X_FRACTION = 0.46
TOP_CENTER_Y_FRACTION = 0.26
TABLE_PLANE_Z = -0.007 + (BARE_TABLE_PLANE_Z - BASE_TABLE_PLANE_Z)  # 카메라 적합이 쓰는 base 좌표계의 상자 윗면 평면


class BoxNotFound(ValueError):
    pass


def red_candidates(bgr, hue_max=None):
    """hue_max: 빨강 색상(OpenCV 0~179)의 위쪽 끝(기본 RED_LOW의 4). 조명에 따라 정육면체 색이 5~6으로 치우치면(2026-10-26 Pi 보정 p2: 중앙값 6)
    기본값으로는 못 찾는다. 더 넓히면 빨간 전선·트레이가 섞이므로 필요한 곳에서만 쓴다(보정 스크립트 6)."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    low = RED_LOW if hue_max is None else (RED_LOW[0], (hue_max, *RED_LOW[1][1:]))
    mask = cv2.inRange(hsv, *low) | cv2.inRange(hsv, *RED_HIGH)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    found = [(tuple(int(v) for v in stats[i][:4]), int(stats[i][4]))
             for i in range(1, count) if stats[i][4] >= MIN_AREA_PX]
    return sorted(found, key=lambda item: -item[1])


def detect_box(bgr, hue_max=None):
    """가장 큰 빨간 덩어리의 bbox와 윗면 중심 픽셀을 돌려준다. 모호하면 거부."""
    found = red_candidates(bgr, hue_max)
    if not found:
        raise BoxNotFound('red box not found in image')
    (x, y, w, h), area = found[0]
    if area > MAX_AREA_PX:
        raise BoxNotFound(f'red region too large for the box: {area}px')
    if len(found) > 1 and found[1][1] > 0.5 * area:
        raise BoxNotFound('more than one red object; refusing to guess')
    height, width = bgr.shape[:2]
    if x <= 2 or y <= 2 or x + w >= width - 2 or y + h >= height - 2:
        raise BoxNotFound('red box touches the image border')
    return dict(bbox=(x, y, w, h), area_px=area,
                pixel=(x + TOP_CENTER_X_FRACTION * w, y + TOP_CENTER_Y_FRACTION * h))


class BoxLocator:
    def __init__(self, transform_path, anchor_path):
        model = json.loads(Path(transform_path).read_text())['model']
        self.R = np.array(model['R'], float)
        self.t = np.array(model['t'], float)
        self.fx, self.fy, self.cx, self.cy = (float(v) for v in model['K'])
        anchor = json.loads(Path(anchor_path).read_text())
        old, grip = np.array(anchor['old_camera_target'], float), np.array(anchor['empirical_anchor'], float)
        # 성공한 집기 위치(grip)와 같은 장면의 카메라 추정(old)의 차이를 보정으로 사용.
        self.offset_xy = (grip - old)[:2]
        self.contact_z = float(grip[2])

    def camera_plane_xy(self, pixel):
        ray = np.array([(pixel[0] - self.cx) / self.fx, (pixel[1] - self.cy) / self.fy, 1.0])
        direction = self.R.T @ ray
        origin = -self.R.T @ self.t
        scale = (TABLE_PLANE_Z - origin[2]) / direction[2]
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError('invalid camera-plane intersection')
        return (origin + scale * direction)[:2]

    def locate(self, bgr):
        box = detect_box(bgr)
        xy = self.camera_plane_xy(box['pixel']) + self.offset_xy
        radius = float(np.hypot(*xy))
        return dict(box, target_xy=(float(xy[0]), float(xy[1])), contact_z=self.contact_z,
                    radius_m=radius)


def annotate(bgr, located):
    out = bgr.copy()
    x, y, w, h = located['bbox']
    cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 0), 2)
    u, v = (int(round(c)) for c in located['pixel'])
    cv2.circle(out, (u, v), 4, (0, 255, 0), -1)
    cv2.putText(out, 'x=%.3f y=%.3f r=%.3f' % (*located['target_xy'], located['radius_m']),
                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)
    return out
