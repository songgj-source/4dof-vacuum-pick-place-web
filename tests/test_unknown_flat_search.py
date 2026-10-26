import unittest
import numpy as np
import test_unknown_mode as support

class FlatSearchTests(unittest.TestCase):
    scene = support.Tests.scene
    detect = support.Tests.detect
    def test_searches_another_patch_when_first_has_a_depth_spike(self):
        rgb, depth, base = self.scene()
        first = self.detect(rgb, depth, base)[0]
        x, y = first['top_center_px']
        depth[y-3:y+4, x-3:x+4] -= 5   # 픽셀 하나의 잡음은 허용하므로, 패치의 5%가 넘는 실제 돌기로 시험한다
        target = self.detect(rgb, depth, base)[0]
        self.assertTrue(target['pickable'], target['problems'])
        self.assertNotEqual(target['top_center_px'], [x,y])

if __name__ == '__main__':
    unittest.main()

class FailedFrameTests(unittest.TestCase):
    def test_timeout_saves_last_camera_frame_without_dispatch(self):
        import tempfile
        from pathlib import Path
        from unittest.mock import patch
        import pick_unknown
        rgb = np.zeros((480, 640, 3), np.uint8)
        depth = np.full((480, 640), 500, np.uint16)
        class Camera:
            def __init__(self, **kw): pass
            def read_synchronized(self): return rgb, depth, {}
            def close(self): pass
        def fail(read, *args):
            read()
            raise TimeoutError('test timeout')
        with tempfile.TemporaryDirectory() as d:
            with (patch.object(pick_unknown, 'ROOT', Path(d)),
                  patch('camera.astra_calibration_camera.AstraCalibrationCamera', Camera),
                  patch.object(pick_unknown, 'median_observation', return_value=(rgb, depth, {})),
                  patch.object(pick_unknown, 'wait_for_new_object', side_effect=fail),
                  patch.object(pick_unknown, 'emit'),
                  patch.object(pick_unknown.subprocess, 'run') as dispatch):
                self.assertEqual(pick_unknown.main(['--wait-new']), 1)
                dispatch.assert_not_called()
            self.assertEqual(len(list(Path(d).rglob('failed_frame_rgb.png'))), 1)
            self.assertEqual(len(list(Path(d).rglob('failed_frame_depth.npy'))), 1)
