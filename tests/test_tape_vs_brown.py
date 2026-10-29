"""어두운 갈색 새 물체를 줄자로 부르던 문제(2026-10-28) 회귀 시험. 색값은 실제 화면에서 잰 값이다."""
import unittest

import cv2
import numpy as np

from perception import object_classify, object_registry


def _label(hsv):
    bgr = np.full((480, 640, 3), (200, 200, 200), np.uint8)
    depth = np.full((480, 640), 600, np.uint16)
    bgr[250:310, 250:310] = cv2.cvtColor(np.full((60, 60, 3), hsv, np.uint8), cv2.COLOR_HSV2BGR)
    depth[250:310, 250:310] = 570
    found = object_classify.classify_objects(bgr, depth)
    return [o['label'] for o in found]


class TapeVersusBrownTest(unittest.TestCase):
    def test_yellow_tape_stays_tape_measure(self):
        self.assertEqual(_label((18, 215, 135)), ['tape_measure'])
        self.assertEqual(_label((18, 200, 95)), ['tape_measure'])      # 그늘진 줄자

    def test_dark_brown_object_is_unknown(self):
        self.assertEqual(_label((17, 133, 61)), ['unknown'])           # 2026-10-28 화면의 갈색 상자
        self.assertEqual(_label((17, 133, 86)), ['unknown'])

    def test_dark_brown_is_not_registration_conflict_with_tape(self):
        sig = dict(kind='color', hue=17., value=61., saturation=133., top_height_mm=30.)     # 갈색은 채도가 낮다
        self.assertFalse(object_registry._like_builtin(sig, 'tape_measure', 30.))
        self.assertTrue(object_registry._like_builtin(dict(sig, saturation=215.), 'tape_measure', 30.))
        self.assertTrue(object_registry._like_builtin(dict(sig, saturation=190., value=90.), 'tape_measure', 30.))   # 어두운 조명의 줄자


if __name__ == '__main__':
    unittest.main()
