import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

CHILD = """
import signal, sys, time
from pathlib import Path
marker = Path(sys.argv[1])
def stop(*_):
    time.sleep(1.0)                      # pick_auto의 정리(게인 복구·hold)처럼 0.25초보다 오래 걸린다
    marker.write_text('cleaned')
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
Path(sys.argv[1] + '.ready').write_text('1')
time.sleep(30)
"""

PARENT = """
import sys
sys.path.insert(0, {src!r})
from motion.graceful_child import run_until_clean_exit
run_until_clean_exit([sys.executable, '-c', {child!r}, sys.argv[1]], {root!r})
"""


class GracefulChildTests(unittest.TestCase):
    def run_parent(self, use_run):
        tmp = tempfile.mkdtemp()
        marker = Path(tmp) / 'marker'
        if use_run:       # 예전 방식: subprocess.run
            code = f"import subprocess, sys; subprocess.run([sys.executable, '-c', {CHILD!r}, sys.argv[1]])"
        else:
            code = PARENT.format(src=str(ROOT / 'src'), child=CHILD, root=str(ROOT))
        parent = subprocess.Popen([sys.executable, '-c', code, str(marker)])
        for _ in range(100):
            if Path(str(marker) + '.ready').exists():
                break
            time.sleep(.05)
        parent.send_signal(signal.SIGINT)          # API /stop과 같다: 바깥 프로세스에만 보낸다
        try:
            parent.wait(10)
        finally:
            if parent.poll() is None:
                parent.kill()
        time.sleep(1.5)
        return marker.exists(), parent.returncode

    def test_old_subprocess_run_cuts_off_cleanup(self):
        cleaned, _ = self.run_parent(use_run=True)
        self.assertFalse(cleaned, 'subprocess.run은 정리 전에 안쪽을 죽인다(이 시험이 문제를 재현한다)')

    def test_helper_waits_for_cleanup_then_stops_outer(self):
        cleaned, code = self.run_parent(use_run=False)
        self.assertTrue(cleaned)
        self.assertNotEqual(code, 0)               # 바깥도 멈춘다(재시도 루프가 다시 움직이지 않게)

    def test_returns_child_exit_code_without_signal(self):
        sys.path.insert(0, str(ROOT / 'src'))
        from motion.graceful_child import run_until_clean_exit
        self.assertEqual(run_until_clean_exit([sys.executable, '-c', 'raise SystemExit(3)'], ROOT), 3)


if __name__ == '__main__':
    unittest.main()
