"""세 칸이 물체로 다 차 트레이 모서리가 가려져도 칸 위치를 구한다(2026-11-02: 정리를 끝낸 뒤 칸 안 세 물체가 '책상에 남은 물체'로 판정됐다)."""
import json
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'scripts'), str(ROOT / 'src')]
import arm_api

DATA = ROOT / 'tests/data/tray_full_20261102'


@pytest.fixture
def root(tmp_path):
    shutil.copytree(ROOT / 'config', tmp_path / 'config')
    shutil.copytree(ROOT / 'output/ik_resume_20261012', tmp_path / 'output/ik_resume_20261012')
    return tmp_path


def _frames():
    depths = np.load(DATA / 'depths.npz')
    slots = {int(k): v for k, v in json.loads((DATA / 'ref_slots.json').read_text()).items()}
    return cv2.imread(str(DATA / 'ref_rgb.png')), depths['ref'], slots, cv2.imread(str(DATA / 'full_rgb.png')), depths['full']


def test_full_tray_that_did_not_move_keeps_the_stored_slots(root):
    ref, ref_depth, slots, full, full_depth = _frames()
    found = arm_api._slots_from_reference(ref, ref_depth, slots, full, full_depth, root)
    assert found is not None and set(found) == {1, 2, 3}
    assert all(abs(found[n]['x_mm'] - slots[n]['x_mm']) < 3 and abs(found[n]['y_mm'] - slots[n]['y_mm']) < 3 for n in slots)


def test_full_tray_that_moved_sideways_follows_the_move(root):
    ref, ref_depth, slots, full, full_depth = _frames()
    shifted = cv2.warpAffine(full, np.float32([[1, 0, 18], [0, 1, 0]]), (full.shape[1], full.shape[0]))
    found = arm_api._slots_from_reference(ref, ref_depth, slots, shifted, full_depth, root)
    assert found is not None
    moved = max(abs(found[n]['x_mm'] - slots[n]['x_mm']) + abs(found[n]['y_mm'] - slots[n]['y_mm']) for n in slots)
    assert moved > 5            # 18픽셀 옆 이동이 칸 위치에 반영된다


def test_blank_picture_gives_no_slots(root):
    ref, ref_depth, slots, full, full_depth = _frames()
    blank = np.full_like(full, 200)
    assert arm_api._slots_from_reference(ref, ref_depth, slots, blank, full_depth, root) is None
