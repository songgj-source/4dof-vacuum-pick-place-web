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


class ChainFlagTests(unittest.TestCase):
    def test_flags(self):
        self.assertEqual([sort_menu.chain_flags(i, 3) for i in range(3)],
                         [['--skip-home-after-place'], ['--skip-home-after-place', '--start-above-tray'], ['--start-above-tray']])
        self.assertEqual(sort_menu.chain_flags(0, 1), [])                # 하나뿐이면 평소처럼(놓은 뒤 HOME)


if __name__ == '__main__':
    unittest.main()
