"""트레이 벽에 붙은 물체가 트레이와 한 덩어리로 묶여 사라지던 문제(2026-10-28) 회귀 시험. 실제 Pi 프레임을 쓴다."""
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from perception import object_classify, object_namer

DATA = ROOT / 'tests/data/classify_20261028'


def _found():
    bgr = cv2.imread(str(DATA / 'desk_rgb.png'))
    depth = np.load(DATA / 'desk_depth_mm.npy')
    return object_classify.classify_objects(bgr, depth)


def test_white_box_touching_the_tray_is_found_separately():
    found = _found()
    white = [o for o in found if o['label'] == 'white_box']
    assert len(white) == 1
    assert max(white[0]['bbox_px'][2:]) < 100 and abs(white[0]['top_height_mm'] - 24) < 4     # 흰 상자 크기·높이(24mm)


def test_other_objects_and_tray_are_unchanged():
    found = _found()
    labels = sorted(o['label'] for o in found)
    assert labels.count('red_cube') == 1
    brown = [o for o in found if o['label'] == 'unknown' and 20 < o['top_height_mm'] < 32 and max(o['bbox_px'][2:]) < 100]
    assert len(brown) == 1 and object_namer.color_name(*brown[0]['color_hsv']) == '갈색'           # 갈색 새 물체
    assert any(max(o['bbox_px'][2:]) > object_classify.MAX_OBJECT_PX for o in found)               # 트레이는 큰 덩어리로 남는다


def test_object_separated_by_a_small_gap_from_a_large_frame():
    bgr = np.full((480, 640, 3), (200, 200, 200), np.uint8)
    depth = np.full((480, 640), 600, np.uint16)
    depth[100:300, 300:310] = 560; depth[100:110, 300:420] = 560; depth[290:300, 300:420] = 560   # 큰 틀(트레이 벽)
    depth[180:240, 236:296] = 576; bgr[180:240, 236:296] = (250, 250, 250)                       # 4px 떨어진 흰 상자
    labels = [o['label'] for o in object_classify.classify_objects(bgr, depth)]
    assert 'white_box' in labels
