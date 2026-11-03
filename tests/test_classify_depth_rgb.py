"""깊이 덩어리에서 색을 읽을 때 깊이-색 어긋남을 맞추는지, 화면 오른쪽 물체도 찾는지, 붙은 정육면체·줄자를 나누는지(2026-11-03)."""
import unittest

import cv2
import numpy as np

from perception import object_classify

RED, TAPE = (0, 220, 150), (18, 215, 135)          # HSV: 빨간 정육면체, 진한 노란 줄자


def scene(blocks):
    """blocks: [(x, y, w, h, hsv, 높이 mm)] 깊이 좌표. 색은 실제 카메라처럼 DEPTH_MINUS_TOP_CENTER_PX만큼 옮겨 그린다."""
    bgr = np.empty((480, 640, 3), np.uint8)
    bgr[...] = cv2.cvtColor(np.full((1, 1, 3), (100, 30, 150), np.uint8), cv2.COLOR_HSV2BGR)    # 채도 낮은 회색빛 책상
    depth = np.full((480, 640), 600, np.uint16)
    dx, dy = (int(round(v)) for v in object_classify.DEPTH_MINUS_TOP_CENTER_PX)
    for x, y, w, h, hsv, top in blocks:
        bgr[y - dy:y + h - dy, x - dx:x + w - dx] = cv2.cvtColor(np.full((h, w, 3), hsv, np.uint8), cv2.COLOR_HSV2BGR)
        depth[y:y + h, x:x + w] = 600 - top
    return bgr, depth


def labels(blocks):
    return sorted(o['label'] for o in object_classify.classify_objects(*scene(blocks)))


class DepthRgbTest(unittest.TestCase):
    def test_object_on_right_side_is_found(self):
        # 예전에는 x 440 오른쪽을 통째로 버려 트레이를 가운데에 두면 오른쪽 물체가 모두 빠졌다
        self.assertEqual(labels([(500, 200, 45, 45, RED, 40)]), ['red_cube'])

    def test_color_is_read_at_the_depth_blob(self):
        found = object_classify.classify_objects(*scene([(300, 200, 45, 45, RED, 40)]))
        self.assertEqual([o['label'] for o in found], ['red_cube'])
        self.assertGreater(found[0]['chromatic_shares']['red_cube'], .9)      # 맞추지 않으면 옆 책상 색이 섞인다

    def test_touching_cube_and_tape_are_split(self):
        # 정육면체와 줄자가 붙어 깊이로는 3600px보다 작은 한 덩어리
        got = labels([(300, 200, 40, 40, RED, 40), (340, 200, 38, 40, TAPE, 33)])
        self.assertEqual(got, ['red_cube', 'tape_measure'])

    def test_small_two_color_blob_stays_as_obstacle(self):
        # 나누면 조각이 최소 크기보다 작다: 나누지 않고 새 물체로 남겨 운반 경로의 장애물로 쓴다(사라지면 안 된다)
        self.assertEqual(labels([(300, 200, 25, 25, RED, 40), (325, 200, 25, 25, TAPE, 33)]), ['unknown'])

    def test_two_color_object_with_wrong_height_is_not_split(self):
        # 빨강·노랑이 섞인 높이 60mm 새 물체 하나: 깊이 높이가 정육면체(40)·줄자(33)와 달라 둘로 쪼개지 않는다
        self.assertEqual(labels([(300, 200, 40, 40, RED, 60), (340, 200, 38, 40, TAPE, 60)]), ['unknown'])


if __name__ == '__main__':
    unittest.main()
