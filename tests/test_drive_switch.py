import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import arm_api


class DriveSwitchTest(unittest.TestCase):
    def make(self):
        return arm_api.Arm(run=lambda *a, **k: 0, read_state=lambda: None, hold=lambda: None)

    def test_drive_command_refused_when_off(self):
        with patch.object(arm_api, 'DRIVE_ENABLED', False):
            status, body = self.make().start('drive', dict(label='책상'))
        self.assertEqual(status, 400)
        self.assertEqual(body['reason_code'], 'drive_disabled')

    def test_status_reports_switch(self):
        with patch.object(arm_api, 'DRIVE_ENABLED', False):
            self.assertIs(self.make().status()['drive_enabled'], False)
        self.assertIs(self.make().status()['drive_enabled'], arm_api.DRIVE_ENABLED)

    def test_base_check_skipped_when_off(self):
        with patch.object(arm_api, 'DRIVE_ENABLED', False):
            self.assertIsNone(arm_api.base_moving())


if __name__ == '__main__':
    unittest.main()


class DriveOffChatTest(unittest.TestCase):
    def test_base_call_never_reaches_server_when_off(self):
        import base_drive
        with patch.dict('os.environ', {'ARM_DRIVE': '0'}):
            with self.assertRaises(base_drive.BaseError) as ctx:
                base_drive.base_state()
        self.assertEqual(ctx.exception.code, 'drive_disabled')

    def test_desk_not_visible_does_not_propose_driving_when_off(self):
        import arm_llm
        arm = arm_api.Arm(run=lambda *a, **k: 0, read_state=lambda: None, hold=lambda: None)
        arm.capture = lambda: (200, dict(objects=[], tray_found=False, in_tray=[]))
        with patch.dict('os.environ', {'ARM_DRIVE': '0'}):
            result = arm_llm.desk_request(arm)
        self.assertIsNone(result['proposal'])
        self.assertIn('주행이 꺼져', result['reply'])
