import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'src'))
import run_sort
import sort_menu

PLAN = dict(label='red_cube', slot_number=2, slot_source='default_rule', manual_target_mm=[230, -10, 40], warnings=[], tray_slots={},
            pick_auto_command=['python', 'pick_auto.py', '--execute', '--supported', '--manual-target', '230', '-10', '40'])


class PlanReuseTests(unittest.TestCase):
    def test_plan_out_saves_without_moving_and_plan_in_skips_the_camera(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'plan.json'
            planned = SimpleNamespace(returncode=0, stdout=json.dumps(PLAN), stderr='')
            with patch.object(run_sort.subprocess, 'run', return_value=planned) as camera, \
                 patch.object(run_sort, 'run_until_clean_exit') as robot, patch('builtins.print'):
                self.assertEqual(run_sort.main(['--execute', '--supported', '--object', 'red_cube', '--plan-out', str(path)]), 0)
            camera.assert_called_once()
            robot.assert_not_called()                                    # 계획만: 로봇은 움직이지 않는다
            self.assertEqual(json.loads(path.read_text())['pick_auto_command'], PLAN['pick_auto_command'])
            with patch.object(run_sort.subprocess, 'run') as camera, \
                 patch.object(run_sort, 'run_until_clean_exit', return_value=0) as robot, patch('builtins.print'):
                self.assertEqual(run_sort.main(['--execute', '--supported', '--object', 'red_cube', '--attempts', '1',
                                                '--plan-in', str(path), '--skip-home-after-place']), 0)
            camera.assert_not_called()                                   # 저장한 계획을 쓴다(사진을 다시 찍지 않는다)
            command = robot.call_args[0][0]
            self.assertEqual(command[:len(PLAN['pick_auto_command'])], PLAN['pick_auto_command'])
            self.assertIn('--skip-home-after-place', command)            # pick_auto에 그대로 넘긴다

    def test_plan_failure_is_reported_like_a_normal_run(self):
        failed = SimpleNamespace(returncode=2, stdout='', stderr='plan_sort.py: error: tray not found')
        printed = []
        with tempfile.TemporaryDirectory() as d, patch.object(run_sort.subprocess, 'run', return_value=failed), \
             patch('builtins.print', side_effect=lambda *a, **k: printed.append(a[0])):
            self.assertEqual(run_sort.main(['--object', 'red_cube', '--plan-out', str(Path(d) / 'p.json')]), 1)
        self.assertIn('planning failed', json.loads(printed[-1])['reason'])

    def test_not_held_retry_takes_a_new_photo(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'plan.json'
            path.write_text(json.dumps(PLAN))
            planned = SimpleNamespace(returncode=0, stdout=json.dumps(PLAN), stderr='')
            with patch.object(run_sort.subprocess, 'run', return_value=planned) as camera, \
                 patch.object(run_sort, 'run_until_clean_exit', side_effect=[3, 0]), patch('builtins.print'):
                self.assertEqual(run_sort.main(['--execute', '--supported', '--object', 'red_cube', '--attempts', '2', '--plan-in', str(path)]), 0)
            camera.assert_called_once()                                  # 첫 시도는 저장 계획, 다시 할 때만 새로 찍는다


class UnregisteredObstacleTests(unittest.TestCase):
    """등록 안 된 물체는 집지 않고 운반 경로 장애물로만 넘긴다."""

    def locator(self):
        import numpy as np
        return SimpleNamespace(fx=1000., fy=1000., cx=0., cy=0., R=np.eye(3), t=np.array([0., 0., 1.]), offset_xy=np.zeros(2))

    def blob(self, label='unknown', px=(250., 0.), size=40, top=35.):
        return dict(label=label, bbox_px=[px[0] - size / 2, px[1] - size / 2, size, size], top_center_px=list(px),
                    height_mm=top - 4, top_height_mm=top)

    def test_only_small_unknown_blobs_in_reach_become_obstacles(self):
        from perception.object_targets import locate_unregistered, DEPTH_MINUS_TOP_CENTER_PX
        dx = DEPTH_MINUS_TOP_CENTER_PX[0]
        found = [self.blob(px=(250. + dx, 5.5)),                  # 작업 범위 안: 장애물
                 self.blob(label='white_box', px=(250. + dx, 5.5)),   # 등록 물체는 따로(locate_targets)
                 self.blob(size=200),                              # 트레이·설비 같은 큰 덩어리
                 self.blob(top=150.),                              # 로봇팔 자신
                 self.blob(px=(50. + dx, 5.5)),                    # 로봇 밑동 근처
                 self.blob(px=(600. + dx, 5.5))]                   # 작업 범위 밖(케이블 등)
        out = locate_unregistered(None, None, ROOT, locator=self.locator(), found=found)
        self.assertEqual(len(out), 1)
        obstacle = out[0]
        self.assertEqual(obstacle['label'], 'unregistered')
        self.assertFalse(obstacle['pickable'])
        self.assertEqual(obstacle['object_geometry']['height_mm'], 38.0)        # 윗면 + 깊이 오차 3mm
        self.assertGreaterEqual(obstacle['object_geometry']['footprint_radius_mm'], 20.)

    def test_build_job_avoids_but_never_picks_unregistered(self):
        from sorting.sort_job import build_job
        obstacle = dict(label='unregistered', target_xy=[.22, .0], object_geometry=dict(height_mm=38., footprint_radius_mm=30.),
                        pickable=False, obstacle_only=True, problems=[])
        with self.assertRaisesRegex(ValueError, 'white_box not found'):
            build_job([obstacle], dict(found=True), {'objects': {}}, 'white_box')

    def test_plan_out_passes_unregistered_count(self):
        with tempfile.TemporaryDirectory() as d:
            planned = SimpleNamespace(returncode=0, stdout=json.dumps(dict(PLAN, unregistered_left=2)), stderr='')
            printed = []
            with patch.object(run_sort.subprocess, 'run', return_value=planned), \
                 patch('builtins.print', side_effect=lambda *a, **k: printed.append(a[0])):
                run_sort.main(['--object', 'red_cube', '--plan-out', str(Path(d) / 'p.json')])
        self.assertEqual(json.loads(printed[-1])['unregistered_left'], 2)

    def test_chain_keeps_largest_unregistered_count(self):
        results = iter([(0, dict(ok=True, unregistered_left=1), 0.), (0, dict(ok=True, unregistered_left=2), 0.),
                        (1, dict(ok=False, reason='tape_measure not found in the current camera view'), 0.),
                        (0, dict(ok=True), 0.), (0, dict(ok=True), 0.)])
        with patch.object(sort_menu, 'newest_result', return_value=None):
            outcome = sort_menu.run_all_chained(run=lambda cmd: next(results))
        self.assertTrue(outcome['ok'])
        self.assertEqual(outcome['unregistered_left'], 2)
        self.assertIn('2개는 그대로', sort_menu.unregistered_note(2))
        self.assertEqual(sort_menu.unregistered_note(0), '')

    def test_collision_reason_is_korean(self):
        text = 'place leg carry could collide with unregistered: held object bottom 20 mm vs object top 38 mm + 10 mm margin'
        self.assertIn('등록 안 된 물체', sort_menu.brief_message(text))


class ChainFlagTests(unittest.TestCase):
    def test_flags(self):
        self.assertEqual([sort_menu.chain_flags(i, 3) for i in range(3)],
                         [['--skip-home-after-place'], ['--skip-home-after-place', '--start-above-tray'], ['--start-above-tray']])
        self.assertEqual(sort_menu.chain_flags(0, 1), [])                # 하나뿐이면 평소처럼(놓은 뒤 HOME)

    def test_stored_tray_plan_switches_to_live_camera_when_tray_moved(self):
        with tempfile.TemporaryDirectory() as directory:
            frame = (Path(directory) / 'rgb.png', Path(directory) / 'depth.npy')
            for path in frame:
                path.touch()
            with patch.object(sort_menu, 'TRAY_FRAME', frame):
                command = sort_menu.sort_command('white_box')
        self.assertIn('--tray-frame', command)
        self.assertIn('--tray-fallback-live', command)

    def test_white_box_does_not_pause_for_post_pick_photo_check(self):
        self.assertNotIn('--hold-check', sort_menu.sort_command('white_box', use_tray_frame=False))


if __name__ == '__main__':
    unittest.main()
