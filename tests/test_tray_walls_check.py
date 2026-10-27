"""벽 방식이 벽선을 추정해 트레이를 잘못 잡으면 칸 바닥 방식으로 바꾼다(2026-10-27 Pi 실제 사진). 장치 없음."""
import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src')]
from tray.tray_detect import detect_tray, detect_tray_checked  # noqa: E402

DATA = ROOT / 'tests/data/tray_20261027'


def test_inferred_wall_layout_is_replaced_by_the_floor_layout():
    bgr, depth = cv2.imread(str(DATA / '20261027T161759_rgb.png')), np.load(DATA / '20261027T161759_depth.npy')
    walls = detect_tray(bgr, depth, ROOT, first_end='far_from_base')
    assert walls['method'] == 'walls'                                    # 예전: 벽선 2개를 추정해 칸 3이 (241, 6)mm
    checked = detect_tray_checked(bgr, depth, ROOT, first_end='far_from_base')
    assert checked['method'] == 'floors'
    slot3 = checked['slots'][3]
    assert math.hypot(slot3['x_mm'] - 157, slot3['y_mm'] + 117) < 20     # 실제 3번 칸(같은 날 다른 사진의 칸 바닥: 157, -117)
    assert any('disagreed' in p for p in checked['problems'])


def test_wrong_wall_guesses_fall_outside_the_cardboard_and_floors_inside():
    # 저장 사진 176장 중 벽 방식이 크게 틀린 두 장(10-26 16:02 87mm 밀림, 10-27 16:17 90도 돌아감)
    from tray.tray_detect import _slots_on_cardboard
    for stamp in ('20261026T160245', '20261027T161759'):
        bgr, depth = cv2.imread(str(DATA / f'{stamp}_rgb.png')), np.load(DATA / f'{stamp}_depth.npy')
        walls = detect_tray(bgr, depth, ROOT, first_end='far_from_base', method='walls')
        floors = detect_tray(bgr, depth, ROOT, first_end='far_from_base', method='floors')
        assert _slots_on_cardboard(bgr, walls, ROOT) is False, stamp
        assert _slots_on_cardboard(bgr, floors, ROOT) is True, stamp
        assert detect_tray_checked(bgr, depth, ROOT, first_end='far_from_base')['method'] == 'floors'


def test_stored_empty_frame_prefers_floors():
    bgr, depth = cv2.imread(str(DATA / '20261027T161759_rgb.png')), np.load(DATA / '20261027T161759_depth.npy')
    assert detect_tray_checked(bgr, depth, ROOT, first_end='far_from_base', prefer_floors=True)['method'] == 'floors'
