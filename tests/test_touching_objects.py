"""서로 붙어 있는 줄자·흰 상자·정육면체가 깊이로는 한 덩어리가 되어 전부 unknown이 되던 문제(2026-10-29 Pi 실제 프레임) 회귀 시험."""
from pathlib import Path

import cv2
import numpy as np

from perception import object_classify

DATA = Path(__file__).resolve().parent / 'data/touching_3_20261029'


def _found():
    return object_classify.classify_objects(cv2.imread(str(DATA / 'desk_rgb.png')), np.load(DATA / 'desk_depth_mm.npy'))


def test_three_touching_objects_are_named_separately():
    found = _found()
    registered = sorted(o['label'] for o in found if o['label'] != 'unknown')
    assert registered == ['red_cube', 'tape_measure', 'white_box']                  # 전에는 세 물체 모두 unknown 한 덩어리였다
    by = {o['label']: o for o in found if o['label'] != 'unknown'}
    assert 35 <= by['red_cube']['top_height_mm'] <= 46 and 26 <= by['tape_measure']['top_height_mm'] <= 40          # 실제 40mm·30mm 근처
    boxes = [o['bbox_px'] for o in by.values()]
    assert all(max(b[2:]) < 100 for b in boxes)                                      # 물체 크기 덩어리(한 덩어리로 합쳐지지 않음)


def test_other_frames_are_not_split():
    # 색이 하나뿐인 물체·트레이는 나누지 않는다: 기존 프레임의 결과가 그대로다
    base = Path(__file__).resolve().parent / 'data'
    found = object_classify.classify_objects(cv2.imread(str(base / 'classify_20261028/desk_rgb.png')), np.load(base / 'classify_20261028/desk_depth_mm.npy'))
    assert sorted(o['label'] for o in found).count('red_cube') == 1
    assert sum(1 for o in found if max(o['bbox_px'][2:]) > object_classify.MAX_OBJECT_PX) >= 1     # 트레이는 큰 덩어리로 남는다
    found = object_classify.classify_objects(cv2.imread(str(base / 'tape_desk_20261028/desk_rgb.png')), np.load(base / 'tape_desk_20261028/desk_depth_mm.npy'))
    assert [o['label'] for o in found if o['label'] == 'tape_measure'] == ['tape_measure']


def test_tape_with_white_marks_is_not_split_into_a_fake_white_box():
    # 2026-10-30 Pi: 줄자 하나의 흰 글씨·반사가 별개의 white_box가 되어 '흰 상자가 둘이라 계획 거부'(작업 실패)가 났다. 한 물체 크기 덩어리는 나누지 않는다
    base = Path(__file__).resolve().parent / 'data/duplicate_white_20261030'
    found = object_classify.classify_objects(cv2.imread(str(base / 'desk_rgb.png')), np.load(base / 'desk_depth_mm.npy'))
    labels = sorted(o['label'] for o in found if o['label'] != 'unknown')
    assert labels == ['tape_measure', 'white_box']                                       # 각각 하나씩
    assert object_classify.SPLIT_MIN_BLOB_PX >= 3600


def test_red_cube_in_shadow_is_still_a_red_cube():
    # 2026-11-02 Pi: 그림자 속 정육면체(밝기 V 중앙값 31)가 색 픽셀 부족으로 '새 물체'가 되어 정리에서 빠졌다
    base = Path(__file__).resolve().parent / 'data/dark_red_20261102'
    found = object_classify.classify_objects(cv2.imread(str(base / 'desk_rgb.png')), np.load(base / 'desk_depth_mm.npy'))
    small = sorted(o['label'] for o in found if max(o['bbox_px'][2:]) < 100)          # 트레이(약 110px 덩어리)는 뺀다
    assert small == ['red_cube', 'tape_measure', 'white_box']


def _white_patch(height_mm):
    bgr = np.full((480, 640, 3), (200, 200, 200), np.uint8)
    depth = np.full((480, 640), 600, np.uint16)
    bgr[250:310, 250:330] = 250
    depth[250:310, 250:330] = 600 - int(height_mm)
    return bgr, depth


def test_white_object_is_a_white_box_only_when_the_height_matches():
    # 2026-11-02: 색만 보고 높이가 다른 흰 상자도 '흰 상자'(카탈로그 높이 24mm)로 집으려 했다
    labels = lambda h: [o['label'] for o in object_classify.classify_objects(*_white_patch(h))]
    assert labels(24) == ['white_box'] and labels(28) == ['white_box']
    assert labels(45) == ['unknown'] and labels(60) == ['unknown']
