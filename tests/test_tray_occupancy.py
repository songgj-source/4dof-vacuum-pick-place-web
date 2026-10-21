import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from tray.occupancy import outside_tray, slot_containing

# 2026-10-21 저장된 빈 트레이에서 찾은 칸 중심(로봇 기준 mm)
SLOTS = {1: dict(x_mm=341.4, y_mm=-130.8), 2: dict(x_mm=229.4, y_mm=-125.1), 3: dict(x_mm=133.4, y_mm=-118.9)}


class OccupancyTests(unittest.TestCase):
    def test_object_inside_a_slot(self):
        self.assertEqual(slot_containing((.140, -.110), SLOTS), 3)          # 3번 칸에 넣은 줄자
        self.assertEqual(slot_containing((.341, -.131), SLOTS), 1)

    def test_objects_on_the_desk_or_beside_the_tray(self):
        self.assertIsNone(slot_containing((.161, -.022), SLOTS))            # 책상 위 줄자(10-21 실측 위치)
        self.assertIsNone(slot_containing((.229, -.060), SLOTS))            # 2번 칸 옆 책상(65mm)
        self.assertIsNone(slot_containing((.2, 0.), None))                   # 트레이를 모르면 빼지 않는다
        self.assertIsNone(slot_containing(None, SLOTS))

    def test_planning_drops_targets_inside_the_tray(self):
        targets = [dict(label='tape_measure', target_xy=(.135, -.120)), dict(label='red_cube', target_xy=(.233, -.010))]
        self.assertEqual([t['label'] for t in outside_tray(targets, SLOTS)], ['red_cube'])


class SlotContentsTests(unittest.TestCase):
    """칸마다 바닥보다 솟은 높이로 무엇이 들어 있는지(가짜 높이 영상, 색은 회색이라 높이만으로 판단)."""
    HEIGHTS = dict(white_box=24., red_cube=40., tape_measure=30.)

    def run_with(self, rises):
        import numpy as np
        from tray.occupancy import slot_contents
        slots = {n: dict(x_mm=float(n), y_mm=0., floor_height_mm=10.) for n in (1, 2, 3)}
        height = np.zeros((480, 640), float)
        bgr = np.full((480, 640, 3), 128, np.uint8)
        pixel = lambda xy, h: (100. + 150. * (xy[0] * 1000. - 1), 200.)       # 칸 1·2·3 → u 100·250·400
        for n, rise in rises.items():
            u = int(100 + 150 * (n - 1))
            height[188:213, u - 12:u + 13] = 10. + rise
        return [(f['slot'], f['label']) for f in slot_contents(bgr, height, slots, pixel, (0., 0.), self.HEIGHTS)]

    def test_height_tells_which_object(self):
        self.assertEqual(self.run_with({1: 25., 2: 38., 3: 32.}), [(1, 'white_box'), (2, 'red_cube'), (3, 'tape_measure')])

    def test_empty_slots_and_odd_heights(self):
        self.assertEqual(self.run_with({2: 5.}), [])                       # 칸막이·숫자 정도는 물체가 아니다
        self.assertEqual(self.run_with({3: 60.}), [(3, 'unknown')])          # 등록 물체 높이와 다르면 알 수 없는 물체

if __name__ == '__main__':
    unittest.main()
