"""지도의 지정한 타원 안 장애물만 빈 공간으로 바꾼다(SLAM 잡음 마스킹, 2026-10-29). 하드웨어 접근 없음."""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import mask_map_region as mask  # noqa: E402


def _map(tmp_path):
    image = np.full((100, 200), 254, np.uint8)          # 10m x 5m, 해상도 0.05
    image[:, 0] = 0                                     # 왼쪽 벽(마스크 밖)
    image[50:52, 100:104] = 0                           # 지도 중심 근처의 점(마스크 안)
    image[10, 150] = 0                                  # 마스크 밖의 다른 점
    image[60, 100] = 205                                # 미탐색은 그대로
    cv2.imwrite(str(tmp_path / 'm.pgm'), image)
    (tmp_path / 'm.yaml').write_text('image: m.pgm\nresolution: 0.050\norigin: [-5.0, -2.5, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    return image


def test_only_obstacles_inside_the_ellipse_become_free(tmp_path):
    original = _map(tmp_path)
    assert mask.main([str(tmp_path / 'm.yaml'), '--ellipse', '0.1', '-0.1', '0.5', '0.3']) == 0
    result = cv2.imread(str(tmp_path / 'm.pgm'), cv2.IMREAD_UNCHANGED)
    assert (result[50:52, 100:104] == 254).all()                           # 점이 지워졌다
    assert (result[:, 0] == 0).all() and result[10, 150] == 0              # 벽·밖의 점은 그대로
    assert result[60, 100] == 205                                          # 미탐색은 그대로
    changed = np.argwhere(result != original)
    assert len(changed) == 8
    note = json.loads((tmp_path / 'm.mask.json').read_text())
    assert note['pixels_changed'] == 8 and note['to'] == 'free'


def test_backup_is_kept_once_and_rerunning_starts_from_the_original(tmp_path):
    original = _map(tmp_path)
    mask.main([str(tmp_path / 'm.yaml'), '--ellipse', '0.1', '-0.1', '0.5', '0.3'])
    mask.main([str(tmp_path / 'm.yaml'), '--ellipse', '0.1', '-0.1', '0.1', '0.1'])        # 더 작은 영역으로 다시: 원본에서 다시 만든다
    assert (cv2.imread(str(tmp_path / 'm.pgm.orig'), cv2.IMREAD_UNCHANGED) == original).all()
    result = cv2.imread(str(tmp_path / 'm.pgm'), cv2.IMREAD_UNCHANGED)
    assert (result == original).sum() > (result != original).sum()
    mask.main([str(tmp_path / 'm.yaml'), '--ellipse', '0.1', '-0.1', '0.5', '0.3', '--unknown'])
    assert (cv2.imread(str(tmp_path / 'm.pgm'), cv2.IMREAD_UNCHANGED)[50:52, 100:104] == 205).all()
