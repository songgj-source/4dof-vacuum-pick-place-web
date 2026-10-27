"""Synthetic perception/tray input through real job/IK/place dry-run; no device access."""
import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'scripts'))
import plan_sort


class UnknownPipelineTest(unittest.TestCase):
    def test_unknown_plan_to_real_pick_auto_dry_run(self):
        geometry = dict(source='depth_table_plane', height_mm=35., footprint_mm=[45., 45.],
                        height_uncertainty_mm=3., footprint_radius_mm=40., top_flat=True, physical_validated=False)
        target = dict(label='unknown', name='unknown', new_object=True, target_xy=[.217, .085],
                      contact_z=.033, radius_m=.233, pickable=True, problems=[], object_geometry=geometry,
                      suction_verified=False, pick_tilt_deg=0.)
        tray = dict(found=True, slots={3: dict(x_mm=180., y_mm=30., floor_height_mm=10., radius_m=.183)},
                    rim_height_mm=40., method='walls', numbering='rule:far_from_base')
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            rgb = np.full((480, 640, 3), 160, np.uint8)
            depth = np.full((480, 640), 500, np.uint16)
            cv2.imwrite(str(directory / 'rgb.png'), rgb)
            np.save(directory / 'depth.npy', depth)
            np.save(directory / 'baseline.npy', depth)
            output = io.StringIO()
            with (patch.object(plan_sort, 'detect_tray', return_value=tray), patch.object(plan_sort, 'detect_tray_checked', return_value=tray),
                  patch('perception.unknown_objects.locate_unknowns', return_value=[target]),
                  patch('tray.tray_capacity.require_slot_capacity', return_value=80.),
                  contextlib.redirect_stdout(output)):
                self.assertEqual(plan_sort.main([
                    '--object', 'unknown', '--slot', '3', '--acknowledge-unverified-suction',
                    '--from-files', str(directory / 'rgb.png'), str(directory / 'depth.npy'),
                    '--unknown-baseline', str(directory / 'baseline.npy'), '--output', str(directory / 'plan'),
                ]), 0)
            plan = json.loads(output.getvalue())
            command = [arg for arg in plan['pick_auto_command'] if arg not in ('--execute', '--supported')]
            self.assertNotIn('--execute', command)
            done = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=15)
            self.assertEqual(done.returncode, 0, done.stderr)
            result = json.loads(done.stdout)
            self.assertFalse(result['motor_writes'])
            self.assertFalse(result['pump_writes'])
            self.assertEqual(result['plan']['place']['label'], 'unknown')
            self.assertEqual(result['plan']['place']['object_geometry']['height_mm'], 35.)


class UnknownCommandTests(unittest.TestCase):
    def test_unknown_command_retakes_the_tray_when_it_moved(self):
        import sort_menu
        with patch.object(sort_menu.Path, 'exists', return_value=True):
            command = sort_menu.unknown_command(3)
        self.assertIn('--tray-fallback-live', command)
        self.assertIn('--tray-frame', command)


if __name__ == '__main__':
    unittest.main()
