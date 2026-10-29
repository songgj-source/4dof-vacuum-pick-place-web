"""물체 위치는 Pi 재보정(camera_transform_objects.json), 트레이 칸은 예전 변환: 두 곳이 갈라져 있는지와 새 변환이 8점을 맞추는지."""
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from perception.object_targets import default_locator  # noqa: E402
from tray.tray_detect import _locator as tray_locator  # noqa: E402

POINTS = ROOT / '카메라 보정/재보정_점_Pi_20261029.jsonl'      # 2026-10-29 재보정(좌우 편차). 2026-10-26 점은 재보정_점_Pi_20261026.jsonl에 남겨 둔다


def test_objects_use_the_pi_recalibration_and_the_tray_keeps_the_old_transform():
    objects, tray = default_locator(ROOT), tray_locator(ROOT)
    assert not np.allclose(objects.R, tray.R) and not np.allclose(objects.t, tray.t)
    old = json.loads((ROOT / 'output/ik_resume_20261012/multipose_review/camera_transform_candidate.json').read_text())['model']
    assert np.allclose(tray.R, old['R']) and np.allclose(tray.t, old['t'])         # 칸 좌표 계산은 그대로
    assert np.allclose(objects.offset_xy, 0)                                       # 새 변환은 앵커 오프셋 0을 전제로 한다


def test_new_transform_reproduces_the_eight_arm_positions():
    locator = default_locator(ROOT)
    rows = [json.loads(line) for line in POINTS.read_text().splitlines()]
    assert len(rows) == 8
    errors = [np.hypot(*((locator.camera_plane_xy(r['pixel']) + locator.offset_xy - np.array(r['arm_fk_xy_m'])) * 1000)) for r in rows]
    assert math.sqrt(sum(e * e for e in errors) / len(errors)) < 3.5               # 맞춤 RMS 2.35mm의 여유
    assert max(errors) < 5.


def test_previous_transform_is_kept_and_was_clearly_off_on_the_new_points():
    # 2026-10-29: 예전(10-26) 변환은 새 점에서 평균 y -18mm 틀려 있었다(좌우 편차). 되돌릴 수 있게 파일을 남긴다
    folder = ROOT / 'output/ik_resume_20261012/multipose_review'
    old = json.loads((folder / 'camera_transform_objects.before_20261029.json').read_text())['model']
    new = json.loads((folder / 'camera_transform_objects.json').read_text())['model']
    assert not np.allclose(old['R'], new['R']) and np.allclose(old['K'], new['K'])
    rows = [json.loads(line) for line in POINTS.read_text().splitlines()]
    from robot.robot_frame import BARE_TABLE_PLANE_Z, BASE_TABLE_PLANE_Z
    plane_z = -0.007 + (BARE_TABLE_PLANE_Z - BASE_TABLE_PLANE_Z)
    R, t, K = np.array(old['R']), np.array(old['t']), old['K']
    def hit(pixel):
        direction = R.T @ np.array([(pixel[0] - K[2]) / K[0], (pixel[1] - K[3]) / K[1], 1.])
        origin = -R.T @ t
        return (origin + (plane_z - origin[2]) / direction[2] * direction)[:2]
    mean = np.mean([(hit(r['pixel']) - np.array(r['arm_fk_xy_m'])) * 1000 for r in rows], axis=0)
    assert abs(mean[1]) > 12                      # 예전 변환의 좌우 편차
