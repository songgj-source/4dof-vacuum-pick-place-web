"""팀 주행 코드 사본(ros2_slam/)과 자비스가 맞물리는지 본다. ROS 없이 코드 글자만 읽는다.

ros2_slam/을 팀 저장소의 새 버전으로 바꿨을 때, 자비스가 부르는 map_web 주소·응답 필드가 사라졌으면 여기서 먼저 깨진다.
"""
import re
import sys
import unittest
from pathlib import Path
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'src'))
import base_drive  # noqa: E402
from camera.astra_calibration_camera import pick_uri  # noqa: E402

SLAM = ROOT / 'ros2_slam'
MAP_WEB = SLAM / 'test' / 'map_web.py'
DRIVE = SLAM / 'test' / 'drive_control.py'


@unittest.skipUnless(MAP_WEB.exists(), 'ros2_slam 사본이 없다')
class MapWebContractTests(unittest.TestCase):
    def setUp(self):
        self.map_web = MAP_WEB.read_text()

    def routes_used_by_jarvis(self):
        used = set(re.findall(r"'(/api/[a-z_/]+)'", (ROOT / 'scripts' / 'base_drive.py').read_text()))
        arm_api = (ROOT / 'scripts' / 'arm_api.py').read_text()
        body = arm_api[arm_api.index('def base_moving'):arm_api.index('class EventLog')]
        return used | set(re.findall(r"'(/api/[a-z_/]+)'", body))

    def test_every_route_jarvis_calls_exists(self):
        routes = self.routes_used_by_jarvis()
        self.assertTrue({'/api/state', '/api/drive', '/api/nav/goto_wall', '/api/wall/config', '/api/get_map_info'} <= routes)
        missing = [r for r in sorted(routes) if f"'{r}'" not in self.map_web]
        self.assertEqual(missing, [], f'map_web.py에 없는 주소: {missing}')

    def test_state_fields_jarvis_reads_exist(self):
        for field in ("'then_state'", "'started'", "'label'", "'target'", "'message'", "distance=", "'gap'"):
            self.assertIn(field, self.map_web, f'map_web.py에 {field}가 없다')
        for state in ('backing', 'sending', 'canceling', 'succeeded', 'aborted', 'rejected', 'unavailable', 'running', 'done', 'failed'):
            self.assertIn(f"'{state}'", self.map_web, f'상태 {state}가 없다')
        drive = DRIVE.read_text()
        for field in ("'moving'", "'driver_connected'", "'odom'"):
            self.assertIn(field, drive, f'drive_control.py에 {field}가 없다')

    def test_goto_wall_takes_a_label_id(self):
        self.assertIn('POST /api/nav/goto_wall {"id": "..."}', self.map_web)

    def test_launched_ports_do_not_clash(self):
        launcher = (ROOT / 'pi' / 'drive_ctl.sh').read_text()
        map_port = int(re.search(r'map_web.py --ros-args -p port:=(\d+)', launcher)[1])
        camera_port = int(re.search(r"declare_parameter\('port', (\d+)\)", (SLAM / 'test' / 'camera_test.py').read_text())[1])
        self.assertEqual(map_port, urlsplit(base_drive.BASE_URL).port)        # 자비스가 기본으로 부르는 곳
        self.assertNotIn(8000, (map_port, camera_port))                       # 자비스 서버
        self.assertNotEqual(map_port, camera_port)


@unittest.skipUnless((SLAM / 'src' / 'car2_driver' / 'car2_driver' / 'serial_probe.py').exists(), 'ros2_slam 사본이 없다')
class LidarAutoPortTests(unittest.TestCase):
    """우리 사본의 라이다 port:=auto: 내용(47바이트 묶음)으로 라이다 포트를 고르고, 펌프 ESP32·사용 중인 포트는 건너뛴다."""

    def setUp(self):
        sys.path.insert(0, str(SLAM / 'src' / 'car2_driver'))
        from car2_driver import serial_probe
        self.probe = serial_probe

    def lidar_bytes(self, packets=6):
        packet = b'\x54\x2c' + bytes(45)
        return b'\x00\x13' + packet * packets

    def test_recognises_lidar_packets_only(self):
        self.assertTrue(self.probe.looks_like_ld(self.lidar_bytes()))
        self.assertFalse(self.probe.looks_like_ld(b'ESP32_VACUUM_L9110S GPIO26/25 ready p/d/s/q\r\n' * 20))
        self.assertFalse(self.probe.looks_like_ld(b''))

    def test_finds_the_lidar_and_skips_busy_ports_and_the_pump(self):
        class Link:
            def __init__(self, data): self.data = data
            def read(self, n): chunk, self.data = self.data[:n], self.data[n:]; return chunk
            def close(self): pass
        streams = {'pump': b'ESP32_VACUUM ready\r\n', 'lidar': self.lidar_bytes()}
        def opener(port):
            if port == 'busy':
                raise OSError('Could not exclusively lock port')
            return Link(streams[port])
        found = self.probe.find_ld_lidar(listen_s=.05, ports=['busy', 'pump', 'lidar'], opener=opener)
        self.assertEqual(found, 'lidar')
        self.assertIsNone(self.probe.find_ld_lidar(listen_s=.05, ports=['pump'], opener=opener))

    def test_launch_defaults_to_auto(self):
        launch = (SLAM / 'src' / 'car2_bringup' / 'launch' / 'robot_lidar.launch.py').read_text()
        self.assertIn("LIDAR_PORT = 'auto'", launch)


class AstraPickTests(unittest.TestCase):
    """팔 카메라(Astra 2bc5/0401)와 팀 카메라(Astra Pro 2bc5/060f)가 같이 꽂혀도 팔 카메라를 고른다."""

    def test_picks_the_arm_camera_among_two(self):
        self.assertEqual(pick_uri([b'2bc5/060f@1/7', b'2bc5/0401@1/12']), b'2bc5/0401@1/12')

    def test_refuses_when_only_the_team_camera_is_plugged(self):
        with self.assertRaises(RuntimeError) as caught:
            pick_uri([b'2bc5/060f@1/7'])
        self.assertIn('ARM_ASTRA_ID', str(caught.exception))

    def test_single_device_without_usb_id_is_used(self):
        self.assertEqual(pick_uri([b'freenect://0']), b'freenect://0')

    def test_no_device_is_a_clear_error(self):
        with self.assertRaises(RuntimeError) as caught:
            pick_uri([])
        self.assertIn('0개', str(caught.exception))

    def test_custom_id(self):
        self.assertEqual(pick_uri(['2bc5/060f@1/7'], wanted='2bc5/060f'), '2bc5/060f@1/7')


if __name__ == '__main__':
    unittest.main()
