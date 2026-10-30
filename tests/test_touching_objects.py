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
