"""사진 위에 집기·놓기 범위를 그린다(2026-10-30 사용자 요청). 하드웨어 접근 없음."""
import math
from pathlib import Path

import cv2
import numpy as np

from perception import reach_overlay
from perception.box_target import TABLE_PLANE_Z
from perception.object_targets import default_locator

ROOT = Path(__file__).resolve().parents[1]
FRAME = ROOT / 'tests/data/duplicate_white_20261030/desk_rgb.png'


def test_inverse_projection_round_trips_the_camera_model():
    locator = default_locator(ROOT)
    for x, y in ((.20, -.05), (.25, .10), (.17, 0.), (.30, -.12)):
        px = reach_overlay.to_pixel(locator, x, y, TABLE_PLANE_Z)
        back = locator.camera_plane_xy(px) + locator.offset_xy
        assert math.hypot(back[0] - x, back[1] - y) < 1e-6          # 사진 픽셀 → 로봇 좌표 → 사진 픽셀이 그대로 돌아온다


def test_overlay_draws_ranges_and_keeps_the_original_untouched():
    bgr = cv2.imread(str(FRAME))
    original = bgr.copy()
    out = reach_overlay.annotate(bgr, ROOT)
    assert out.shape == bgr.shape and (bgr == original).all()        # 원본은 그대로, 복사본에 그린다
    assert (out != bgr).any()
    green = ((out[..., 1] > 150) & (out[..., 0] < 120) & (out[..., 2] < 140)).sum()
    blue = ((out[..., 0] > 200) & (out[..., 1] > 120) & (out[..., 2] < 90)).sum()
    assert green > 300 and blue > 300                                  # 초록 띠·선, 파랑 놓기 선


def test_objects_and_slots_get_distance_labels():
    bgr = cv2.imread(str(FRAME))
    plain = reach_overlay.annotate(bgr, ROOT)
    scene = dict(objects=[dict(label='tape_measure', pixel=[180, 290], position_mm=[200, 30], pickable=True),
                          dict(label='white_box', pixel=[240, 240], position_mm=[290, -10], pickable=False)],
                 slot_radius_mm={1: 382., 2: 360., 3: 345.}, slot_angle_rad={1: .1, 2: 0., 3: -.1})
    out = reach_overlay.annotate(bgr, ROOT, scene)
    assert (out != plain).any()
    px = reach_overlay.to_pixel(default_locator(ROOT), .2, .03, TABLE_PLANE_Z)
    assert px is not None


def test_failure_never_breaks_the_photo_and_can_be_turned_off(tmp_path):
    bgr = cv2.imread(str(FRAME))
    out = reach_overlay.annotate(bgr, tmp_path / 'missing')            # 보정 파일이 없어도 사진은 그대로 나간다
    assert (out == bgr).all()
    source = (ROOT / 'scripts/arm_api.py').read_text()
    assert "os.environ.get('ARM_REACH_OVERLAY', '1') != '0'" in source and 'reach_overlay.annotate(bgr, menu.ROOT, scene)' in source
    assert '초록 띠: 집을 수 있는 거리' in (ROOT / 'web/index.html').read_text()
