"""물체 윗면 중심이 안 닿아도 윗면의 다른 곳이 닿으면 거기를 집는다(2026-10-30 사용자 요청). 하드웨어 접근 없음."""
import math
from pathlib import Path

import cv2
import numpy as np

from perception import object_targets as ot

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / 'tests/data'


def test_shift_limit_keeps_the_cup_on_the_top_face():
    shrink = ot.CUP_RADIUS_MM + ot.CUP_EDGE_MARGIN_MM
    cube = dict(footprint_mm=[40, 40], top_flat=True)
    assert ot.max_pick_shift_mm((1., 0.), cube, None) == 20 - shrink                       # 방향을 모르면 짧은 변 기준
    aligned = dict(axis_xy=[1., 0.], half_mm=[20., 20.])
    rotated = dict(axis_xy=[math.cos(math.pi / 4), math.sin(math.pi / 4)], half_mm=[20., 20.])
    assert ot.max_pick_shift_mm((1., 0.), cube, aligned) == 20 - shrink
    assert abs(ot.max_pick_shift_mm((1., 0.), cube, rotated) - (20 - shrink) / math.cos(math.pi / 4)) < 1e-6      # 45° 돌아 있으면 대각선 방향으로 더 간다
    box = dict(footprint_mm=[50, 70], top_flat=True)
    along_long = dict(axis_xy=[1., 0.], half_mm=[35., 25.])
    assert ot.max_pick_shift_mm((1., 0.), box, along_long) == 35 - shrink                    # 긴 변 방향
    assert ot.max_pick_shift_mm((0., 1.), box, along_long) == 25 - shrink                    # 짧은 변 방향
    assert ot.max_pick_shift_mm((1., 0.), dict(footprint_mm=[40, 40], top_flat=False), None) == 0.     # 평평하지 않으면 옮기지 않는다
    assert ot.max_pick_shift_mm((1., 0.), dict(), None) == 0.                                 # 크기를 모르면 옮기지 않는다
    assert ot.max_pick_shift_mm((1., 0.), dict(footprint_mm=[20, 20], top_flat=True), None) == 0.   # 컵보다 작은 윗면


def _targets(folder):
    bgr = cv2.imread(str(DATA / folder / 'desk_rgb.png'))
    depth = np.load(DATA / folder / 'desk_depth_mm.npy')
    return {t['label']: t for t in ot.locate_targets(bgr, depth, ROOT)}


def test_white_box_just_beyond_the_reach_is_picked_off_center():
    # 2026-10-30: 중심 28.2cm는 '도달 불가'였는데 윗면 안쪽 3mm 지점은 닿는다
    white = _targets('touching_3_20261029')['white_box']
    assert white['pickable'] and white['pick_offset_mm'] <= -ot.SHIFT_STEP_MM + 1e-9 and white['pick_offset_mm'] >= -12
    assert abs(math.hypot(*white['center_xy']) - math.hypot(*white['target_xy']) - abs(white['pick_offset_mm']) / 1000.) < 1e-6     # 안쪽(로봇 쪽)으로 옮김
    assert white['radius_m'] < math.hypot(*white['center_xy'])


def test_centered_objects_are_not_moved_and_hopeless_ones_stay_unpickable():
    targets = _targets('touching_3_20261029')
    assert targets['red_cube']['pick_offset_mm'] == 0. and targets['tape_measure']['pick_offset_mm'] == 0.      # 중심이 닿으면 중심을 집는다
    far = _targets('duplicate_white_20261030')['white_box']                                   # 중심 29.2cm: 컵이 윗면 안에 머무는 범위로도 부족
    assert not far['pickable'] and far['pick_offset_mm'] == 0.


def test_scene_reports_the_offset_and_overlay_marks_it():
    src = (ROOT / 'scripts/arm_api.py').read_text()
    assert "pick_offset_mm=round(float(target.get('pick_offset_mm') or 0.), 1)" in src
    overlay = (ROOT / 'src/perception/reach_overlay.py').read_text()
    assert "'*' if abs(obj.get('pick_offset_mm') or 0.) >= .5" in overlay
