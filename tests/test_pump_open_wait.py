"""펌프 포트를 라이다 탐색이 잠깐 잡고 있으면 기다렸다 연다(2026-10-27 Pi). 장치 없음."""
import sys
from pathlib import Path
from unittest.mock import patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src')]
from hardware import vacuum_serial  # noqa: E402

PUMP = '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3.2:1.0-port0'


def open_with(owner_checks):
    checks = iter(owner_checks)
    def require(device):
        if next(checks):
            raise OSError('motor port already owned by another open handle: 1999')
    opened = []
    with patch.object(vacuum_serial, 'require_device_available', side_effect=require), \
         patch.object(vacuum_serial, 'open_exclusive_serial', side_effect=lambda *a: opened.append(a) or object()), \
         patch.object(vacuum_serial.SerialVacuum, '_wait_ready'), patch.object(vacuum_serial.SerialVacuum, 'stop'), \
         patch('hardware.devices.configured', return_value=PUMP), patch('hardware.devices.stable_serial', return_value=True), \
         patch.object(vacuum_serial.time, 'sleep'):
        vacuum_serial.SerialVacuum(PUMP).open()
    return opened


def test_pump_waits_while_the_lidar_probe_briefly_holds_the_port():
    assert len(open_with([True, True, False])) == 1


def test_pump_still_refuses_a_port_that_stays_owned():
    clock = iter(range(0, 100))
    with patch.object(vacuum_serial.time, 'monotonic', side_effect=lambda: next(clock)):
        with pytest.raises(OSError, match='already owned'):
            open_with([True] * 100)
