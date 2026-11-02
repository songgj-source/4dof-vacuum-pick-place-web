"""칸 위치를 직접 못 찾을 때(칸 안 물체가 바닥을 가림) 가장 최근의 빈 트레이를 기준으로 맞춘다(2026-11-02: 트레이를 옮긴 뒤 칸 안 물체가 책상 물체로 잡혔다)."""
import shutil
import sys
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'scripts'), str(ROOT / 'src')]
import arm_api
from tray import tray_detect

DATA = ROOT / 'tests/data/dark_red_20261102'


@pytest.fixture
def workdir(tmp_path):
    shutil.copytree(ROOT / 'config', tmp_path / 'config')
    shutil.copytree(ROOT / 'output/ik_resume_20261012', tmp_path / 'output/ik_resume_20261012')
    (tmp_path / 'output/tray_frames').mkdir(parents=True)
    frames = tmp_path / 'output/tray_frames'
    with patch.object(arm_api.menu, 'ROOT', tmp_path), patch.object(arm_api.menu, 'TRAY_FRAME', (frames / 'latest_rgb.png', frames / 'latest_depth.npy')):
        yield tmp_path


def _frame():
    return cv2.imread(str(DATA / 'desk_rgb.png')), np.load(DATA / 'desk_depth_mm.npy')


def test_empty_tray_found_live_is_remembered(workdir):
    rgb, depth = _frame()
    slots = arm_api.tray_slots(rgb, depth)
    assert slots and set(slots) == {1, 2, 3}
    assert all((workdir / 'output/tray_frames' / name).exists() for name in arm_api.LAST_TRAY_FILES)


def test_tray_with_objects_in_it_is_not_remembered(workdir):
    rgb, depth = _frame()
    with patch('tray.occupancy.slot_contents', return_value=[dict(slot=2, label='tape_measure')]):
        assert arm_api.tray_slots(rgb, depth)
    assert not any((workdir / 'output/tray_frames' / name).exists() for name in arm_api.LAST_TRAY_FILES)


def test_moved_tray_with_hidden_floors_follows_the_last_empty_tray(workdir):
    rgb, depth = _frame()
    before = arm_api.tray_slots(rgb, depth)
    shift = np.float32([[1, 0, 24], [0, 1, 0]])                      # 트레이가 사진에서 오른쪽으로 24px 옮겨졌다
    moved_rgb = cv2.warpAffine(rgb, shift, (640, 480), borderMode=cv2.BORDER_REPLICATE)
    moved_depth = cv2.warpAffine(depth, shift, (640, 480), flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_REPLICATE)
    with patch.object(tray_detect, 'detect_tray_checked', return_value=dict(found=False)) as detect:
        # 칸 바닥을 못 찾는 사진: 저장해 둔 빈 트레이 두 곳 모두 이 사진과 맞춰 본다
        after = arm_api.tray_slots(moved_rgb, moved_depth)
    assert detect.called and after is not None
    for number in before:                                            # 옮겨진 만큼(24px = 로봇 옆 방향 약 30mm) 칸 위치가 따라 움직인다. 그대로(옛 위치)도, 못 구함도 아니다
        assert abs(after[number]['x_mm'] - before[number]['x_mm']) < 6
        assert 22 < abs(after[number]['y_mm'] - before[number]['y_mm']) < 40


def test_unmatched_picture_gives_none_as_before(workdir):
    rgb, depth = _frame()
    arm_api.tray_slots(rgb, depth)
    shift = np.float32([[1, 0, 90], [0, 1, 0]])                      # 너무 크게 옮겨져 맞출 수 없다
    far_rgb = cv2.warpAffine(rgb, shift, (640, 480), borderMode=cv2.BORDER_REPLICATE)
    far_depth = cv2.warpAffine(depth, shift, (640, 480), flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_REPLICATE)
    with patch.object(tray_detect, 'detect_tray_checked', return_value=dict(found=False)):
        assert arm_api.tray_slots(far_rgb, far_depth) is None
