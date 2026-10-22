"""Raspberry Pi 이전용: 장치 경로 설정, 원격 음성 인식, 노트북 음성 서버, 설치 스크립트(실행 안 함 모드). 로봇·카메라 없음."""
import json
import os
import subprocess
import sys
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'src'))
from hardware import devices          # noqa: E402
import arm_stt                        # noqa: E402
import stt_server                     # noqa: E402


class DevicePathTests(unittest.TestCase):
    def setUp(self):
        for name in devices.ENV.values():
            patcher = patch.dict(os.environ, {name: ''})
            patcher.start(); self.addCleanup(patcher.stop)
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        patcher = patch.object(devices, 'CONFIG', Path(self.tmp.name) / 'devices.json')
        patcher.start(); self.addCleanup(patcher.stop)

    def test_defaults_are_the_laptop_values(self):
        self.assertIn('CP2102', devices.get('pump_port'))
        self.assertIsNone(devices.get('servo_port'))                 # 프로필(servo_driver_candidate.json) 값을 쓴다
        self.assertEqual(devices.get('servo_port', 'from-profile'), 'from-profile')
        self.assertFalse(devices.configured('pump_port'))

    def test_file_then_environment_override(self):
        devices.CONFIG.write_text(json.dumps({'pump_port': '/dev/serial/by-id/from-file', 'unknown_key': 'x'}))
        self.assertEqual(devices.lookup('pump_port'), ('/dev/serial/by-id/from-file', 'file'))
        with patch.dict(os.environ, {'ARM_PUMP_PORT': '/dev/serial/by-id/from-env'}):
            self.assertEqual(devices.lookup('pump_port'), ('/dev/serial/by-id/from-env', 'env'))
            self.assertTrue(devices.configured('pump_port'))
        devices.CONFIG.write_text('not json')
        self.assertEqual(devices.lookup('pump_port')[1], 'default')     # 깨진 설정 파일은 무시한다

    def test_by_path_is_allowed_only_when_configured(self):
        by_id, by_path = Path(self.tmp.name) / 'by-id', Path(self.tmp.name) / 'by-path'
        for folder in (by_id, by_path):
            folder.mkdir(); (folder / 'dev').symlink_to('/dev/null')
        with patch.object(devices, 'SERIAL_STABLE_DIRS', (by_id, by_path)):
            self.assertTrue(devices.stable_serial(by_id / 'dev', configured=False))
            self.assertFalse(devices.stable_serial(by_path / 'dev', configured=False))   # 기본값은 by-id만
            self.assertTrue(devices.stable_serial(by_path / 'dev', configured=True))     # 직접 지정하면 포트 위치 이름도
            self.assertFalse(devices.stable_serial(Path('/dev/ttyUSB0'), configured=True))

    def test_configured_pump_must_still_be_a_stable_name(self):
        from hardware.vacuum_serial import SerialVacuum
        with patch.dict(os.environ, {'ARM_PUMP_PORT': '/dev/ttyUSB7'}):
            with self.assertRaisesRegex(ValueError, 'ARM_PUMP_PORT'):
                SerialVacuum(devices.get('pump_port')).open()             # /dev/serial/by-id가 아니면 거부(장치는 열지 않는다)


class RemoteSttTests(unittest.TestCase):
    def serve(self, token='s3', transcriber=None):
        server = ThreadingHTTPServer(('127.0.0.1', 0), stt_server.make_handler(token, transcriber))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close); self.addCleanup(server.shutdown)
        return f'http://127.0.0.1:{server.server_address[1]}/api/stt'

    def remote(self, url, token):
        for name, value in (('REMOTE_URL', url), ('REMOTE_TOKEN', token)):
            patcher = patch.object(arm_stt, name, value); patcher.start(); self.addCleanup(patcher.stop)

    def test_forwards_audio_and_returns_text(self):
        seen = []
        url = self.serve(transcriber=lambda audio, suffix: seen.append((audio, suffix)) or dict(text=' 빨간 큐브 넣어줘. ', seconds=.9))
        self.remote(url, 's3')
        result = arm_stt.transcribe(b'x' * 2000, suffix='.ogg')
        self.assertEqual((result['text'], result['remote']), ('빨간 큐브 넣어줘', True))
        self.assertEqual(seen, [(b'x' * 2000, '.ogg')])
        self.assertIn('원격 Whisper', arm_stt.describe())
        arm_stt.load()                                                    # /health 확인만 하고 모델은 불러오지 않는다

    def test_wrong_token_and_dead_server_are_reported_as_unavailable(self):
        self.remote(self.serve(transcriber=lambda audio, suffix: dict(text='x', seconds=0.)), 'wrong')
        with self.assertRaises(arm_stt.SttUnavailable):
            arm_stt.transcribe(b'x' * 2000)
        self.remote('http://127.0.0.1:9/api/stt', '')
        with self.assertRaises(arm_stt.SttUnavailable):
            arm_stt.transcribe(b'x' * 2000)
        with self.assertRaises(arm_stt.SttUnavailable):
            arm_stt.load()

    def test_stt_server_checks_size_and_never_forwards_again(self):
        url = self.serve(transcriber=lambda audio, suffix: dict(text='ok', seconds=0.))
        request = urllib.request.Request(url, data=b'', method='POST', headers={'Authorization': 'Bearer s3'})
        with self.assertRaises(urllib.error.HTTPError) as caught:
            urllib.request.urlopen(request, timeout=5)
        self.assertEqual(caught.exception.code, 400)
        self.assertNotIn('ARM_STT_URL', os.environ)                       # 노트북 서버는 자기 자신에게 넘기지 않는다

    def test_api_server_uses_the_remote_when_configured(self):
        import arm_api
        url = self.serve(transcriber=lambda audio, suffix: dict(text='멈춰', seconds=.1))
        self.remote(url, 's3')
        server = ThreadingHTTPServer(('127.0.0.1', 0), arm_api.make_handler(arm_api.Arm(run=lambda *a, **k: (0, {}, 0.), read_state=lambda: 'ok'), 't'))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close); self.addCleanup(server.shutdown)
        request = urllib.request.Request(f'http://127.0.0.1:{server.server_address[1]}/api/stt', data=b'x' * 2000, method='POST',
                                         headers={'Authorization': 'Bearer t', 'Content-Type': 'audio/webm'})
        with urllib.request.urlopen(request, timeout=10) as response:
            body = json.loads(response.read())
        self.assertEqual((body['text'], body['remote']), ('멈춰', True))


class ShutdownTests(unittest.TestCase):
    """서버를 끌 때 실행 중인 작업을 정리하고 끝낸다(백그라운드 실행에서는 서버만 신호를 받는다)."""

    def make_arm(self, cleanup_s):
        import arm_api, time
        interrupted = threading.Event()
        class Proc:
            def poll(self): return None
            def send_signal(self, _): interrupted.set()
        def run(command, on_event=None, on_start=None, quiet=False):
            on_start(Proc())
            interrupted.wait(10)
            time.sleep(cleanup_s)                    # pick_auto의 정리(게인 복구·팔 고정)에 걸리는 시간
            return 130, dict(ok=False, reason='cancelled'), 0.
        arm = arm_api.Arm(run=run, read_state=lambda: 'ok')
        with patch.object(arm_api.menu, 'live_view_running', return_value=False):
            arm.start('return_home', {})
            time.sleep(.2)
        return arm, interrupted

    def test_idle_server_just_exits(self):
        import arm_api
        self.assertTrue(arm_api.shutdown_arm(arm_api.Arm(run=lambda *a, **k: (0, {}, 0.), read_state=lambda: 'ok')))

    def test_running_task_gets_the_signal_and_is_waited_for(self):
        import arm_api
        arm, interrupted = self.make_arm(cleanup_s=.6)
        self.assertEqual(arm.status()['state'], 'busy')
        self.assertTrue(arm_api.shutdown_arm(arm, wait_s=5))
        self.assertTrue(interrupted.is_set())                       # 작업에 정지가 전달됐다
        self.assertEqual(arm.status()['state'], 'idle')             # 정리가 끝난 뒤에 돌아왔다

    def test_gives_up_after_the_timeout(self):
        import arm_api
        arm, _ = self.make_arm(cleanup_s=3)
        self.assertFalse(arm_api.shutdown_arm(arm, wait_s=.5))


class InstallScriptTests(unittest.TestCase):
    def test_dry_run_changes_nothing_and_renders_the_service(self):
        script = ROOT / 'pi' / 'install.sh'
        self.assertEqual(subprocess.run(['bash', '-n', str(script)]).returncode, 0)
        done = subprocess.run(['bash', str(script), '--openni2', '/opt/none'], capture_output=True, text=True,
                              env={**os.environ, 'DRY_RUN': '1'}, timeout=60)
        self.assertEqual(done.returncode, 0, done.stderr)
        self.assertIn(f'WorkingDirectory={ROOT}', done.stdout)
        self.assertIn('KillSignal=SIGINT', done.stdout)
        self.assertRegex(done.stdout, r'ARM_API_TOKEN=[0-9a-f]{24}')
        self.assertNotIn('@ROOT@', done.stdout)
        self.assertFalse(Path('/etc/arm-api.env').exists() and 'pitest' in done.stdout)


if __name__ == '__main__':
    unittest.main()
