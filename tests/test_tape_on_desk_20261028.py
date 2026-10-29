"""책상 위 줄자를 '갈색 자석 같은 새 물체'로, 트레이 조각을 새 물체로 부르던 문제(2026-10-28) 회귀 시험. 실제 Pi 프레임을 쓴다.
사진: 빨간 정육면체 1번 칸, 흰 상자 2번 칸, 노란 줄자는 트레이 밖 책상 위. 이 사진에서는 트레이를 찾지 못했다."""
from pathlib import Path

import cv2
import numpy as np

from perception import object_classify
from tray import occupancy

DATA = Path(__file__).resolve().parent / 'data/tape_desk_20261028'


def _found():
    return object_classify.classify_objects(cv2.imread(str(DATA / 'desk_rgb.png')), np.load(DATA / 'desk_depth_mm.npy'))


def test_tape_under_current_light_is_tape_measure():
    tape = [o for o in _found() if o['label'] == 'tape_measure']
    assert len(tape) == 1 and abs(tape[0]['top_height_mm'] - 34) < 4


def test_tray_wall_pieces_are_not_new_objects():
    unknown = [o for o in _found() if o['label'] == 'unknown' and o['top_height_mm'] >= 16.]      # 16mm 미만은 바닥 잡음(세지 않음)
    assert unknown == []


def test_occupancy_red_and_tape_rules():
    # 칸 안 색 판정: 진한 빨강은 정육면체, 채도 낮은 붉은 기운(판지·그늘)은 아니다. 줄자는 진한 노랑만
    patch = np.array([[2, 200, 150], [4, 90, 120], [18, 215, 135], [17, 133, 86]], np.uint8)
    fr = occupancy._fractions(patch)
    assert fr['red_cube'] == .25 and fr['tape_measure'] == .25
