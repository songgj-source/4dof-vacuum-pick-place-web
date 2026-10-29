#!/usr/bin/env python3
"""지도(PGM) 안의 타원 영역에 있는 장애물(검은 점)을 빈 공간으로 바꾼다. SLAM 잡음(움직인 사람·물건)이 남긴 가짜 장애물을 지울 때 쓴다.

  python scripts/mask_map_region.py ros2_slam/maps/map_20261029_1438.yaml --ellipse 0.05 -0.9 0.65 0.5

--ellipse X Y RX RY: 지도 좌표(m)의 중심과 반지름. 여러 번 줄 수 있다. 기본은 장애물(0)만 빈 공간(254)으로 바꾸고, 미탐색(205)·빈 공간은 그대로 둔다.
--unknown: 빈 공간 대신 미탐색(205)으로 바꾼다. 원본은 <이름>.pgm.orig로 한 번만 백업한다(이미 있으면 그것이 원본이다).
지도 yaml의 origin·resolution으로 좌표를 환산한다. .posegraph(SLAM 이어서 하기용)는 건드리지 않는다.
"""
import argparse
import json
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

FREE, UNKNOWN, OCCUPIED = 254, 205, 0


def ellipse_mask(shape, origin, resolution, center, radii):
    """지도 좌표 타원(center, radii in m) 안의 픽셀 마스크. PGM의 0행은 지도 위쪽(y 최대)이다."""
    rows, cols = shape
    ys, xs = np.mgrid[0:rows, 0:cols]
    x = origin[0] + (xs + .5) * resolution
    y = origin[1] + (rows - 1 - ys + .5) * resolution
    return ((x - center[0]) / radii[0]) ** 2 + ((y - center[1]) / radii[1]) ** 2 <= 1.


def mask_regions(image, origin, resolution, ellipses, value=FREE):
    out = image.copy()
    total = 0
    for cx, cy, rx, ry in ellipses:
        inside = ellipse_mask(image.shape, origin, resolution, (cx, cy), (rx, ry))
        hit = inside & (out == OCCUPIED)
        out[hit] = value
        total += int(hit.sum())
    return out, total


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('map_yaml', type=Path)
    parser.add_argument('--ellipse', nargs=4, type=float, action='append', metavar=('X', 'Y', 'RX', 'RY'), required=True)
    parser.add_argument('--unknown', action='store_true')
    args = parser.parse_args(argv)
    info = yaml.safe_load(args.map_yaml.read_text())
    pgm = args.map_yaml.parent / info['image']
    image = cv2.imread(str(pgm), cv2.IMREAD_UNCHANGED)
    if image is None or image.ndim != 2:
        parser.error(f'cannot read a grayscale map: {pgm}')
    backup = pgm.with_suffix('.pgm.orig')
    if not backup.exists():
        shutil.copy2(pgm, backup)
    original = cv2.imread(str(backup), cv2.IMREAD_UNCHANGED)           # 항상 원본에서 다시 만든다(여러 번 돌려도 같은 결과)
    out, count = mask_regions(original, info['origin'][:2], float(info['resolution']), args.ellipse, UNKNOWN if args.unknown else FREE)
    cv2.imwrite(str(pgm), out)
    note = args.map_yaml.with_suffix('.mask.json')
    note.write_text(json.dumps(dict(ellipses_m=args.ellipse, to='unknown' if args.unknown else 'free', pixels_changed=count, original=backup.name), ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(pixels_changed=count, backup=str(backup), note=str(note)), ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
