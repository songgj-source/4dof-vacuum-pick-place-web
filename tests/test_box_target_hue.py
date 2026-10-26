"""빨간 상자 검출의 색상 상한(hue_max): 기본은 그대로, 보정 스크립트만 넓혀 쓴다."""
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from perception.box_target import BoxNotFound, detect_box  # noqa: E402


def scene(hue):
    hsv = np.zeros((480, 640, 3), np.uint8)
    hsv[..., 1], hsv[..., 2] = 10, 200                          # 흰 책상
    hsv[300:350, 200:250] = (hue, 200, 60)                      # 어두운 빨간 상자 50x50
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def test_default_range_is_unchanged():
    assert detect_box(scene(3))['bbox'][:2] == (200, 300)
    with pytest.raises(BoxNotFound):
        detect_box(scene(6))                                    # 색상 6은 기본값(4)으로는 못 찾는다


def test_calibration_range_finds_hue_6_without_moving_the_box():
    box = detect_box(scene(6), hue_max=6)
    assert box['bbox'][:2] == (200, 300) and abs(box['bbox'][2] - 50) <= 1
    assert detect_box(scene(3), hue_max=6)['bbox'] == detect_box(scene(3))['bbox']
