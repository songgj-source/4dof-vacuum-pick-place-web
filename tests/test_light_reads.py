"""느린 이동(하강·들기)의 가벼운 읽기: 틱마다 36번 왕복하던 전체 읽기를 관절당 한 번으로 줄인다(2026-10-26 Pi 떨림). 장치 없음."""
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src')]
from hardware.servo_transport import ServoTransport  # noqa: E402
from motion.hover_trial import _execute_path, TRAVEL_GOAL_SPEED  # noqa: E402

NAMES = ('base', 'shoulder', 'elbow', 'wrist_flex')
FULL = {3: 777, 9: 600, 11: 3500, 31: 0, 33: 0, 40: 1, 42: 2000, 56: 2000, 65: 0}


def transport(torque=1):
    bus = ServoTransport.__new__(ServoTransport)
    bus.profile = SimpleNamespace(joints=[SimpleNamespace(name=n, motor_id=i + 1) for i, n in enumerate(NAMES)])
    bus.port = object()
    bus._motion_static = None
    bus.read_retry_count, bus.read_retry_events = 0, []
    bus.calls = []

    def register(motor_id, address, size):
        bus.calls.append((motor_id, address, size))
        if (address, size) == (33, 34):                   # 모드(33)부터 34바이트: 토크 40, 목표 42, 위치 56, 알람 65
            raw = bytearray(34)
            raw[7] = torque
            raw[9:11] = (2000).to_bytes(2, 'little')
            raw[23:25] = (2003).to_bytes(2, 'little')
            return int.from_bytes(raw, 'little')
        return torque if address == 40 else FULL[address]
    bus._read_register = register
    return bus


def test_light_reads_use_one_read_per_joint_and_keep_static_checks():
    bus = transport()
    bus.read()
    assert len(bus.calls) == 36                          # 지금까지의 틱 읽기
    bus.begin_light_reads()
    bus.calls.clear()
    data = bus.read()
    assert len(bus.calls) == 4 and all(c[1:] == (33, 34) for c in bus.calls)
    row = data['joints']['shoulder']
    assert (row['model'], row['min_limit'], row['max_limit'], row['homing']) == (777, 600, 3500, 0)   # 시작할 때 확인한 고정값
    assert (row['mode'], row['torque'], row['goal'], row['position']) == (0, 1, 2000, 2003)            # 동적 값은 매번 새로
    bus.end_position_stream()
    bus.calls.clear()
    bus.read()
    assert len(bus.calls) == 36                          # 끝나면 원래 전체 읽기


def test_light_reads_refuse_without_torque():
    with pytest.raises(ValueError, match='torque ON'):
        transport(torque=0).begin_light_reads()


class FakeBus:
    def __init__(self):
        self.events = []

    def begin_light_reads(self): self.events.append('light')
    def begin_position_stream(self, origin, speed, acceleration): self.events.append(('stream', speed))
    def end_position_stream(self): self.events.append('end')
    def sync_goal_positions(self, raw): self.driver.position = dict(raw)
    def goal(self, name, raw, speed=40): self.driver.position[name] = raw


class FakeDriver:
    def __init__(self, bus):
        self.names, self.bus, self.state, self.hold_confirmed = NAMES, bus, 'hold_only', True
        bus.driver = self
        self.position = {n: 2000 for n in NAMES}
        self.t = 0.

    def clock(self): return self.t
    def sleep(self, s): self.t += max(0., s)
    def hold(self): pass
    def _fault(self, exc): self.state = 'fault'

    def _snapshot(self, torque=1, fault_on_error=True):
        self.t += .003
        return {'joints': {n: {'position': self.position[n], 'goal': self.position[n]} for n in NAMES}}


def move(speed, period_s):
    bus = FakeBus()
    driver = FakeDriver(bus)
    joints = {n: SimpleNamespace(raw_min=0, raw_max=4095) for n in NAMES}
    origin = {n: 2000 for n in NAMES}
    goal = dict(origin, shoulder=2040)
    result = _execute_path(driver, SimpleNamespace(forward=lambda q: (0., 0., 0., -math.pi / 2)), joints, lambda raw: raw,
                           lambda raw: None, origin, goal, (0., 0., 0.), lambda e: None, lambda: False,
                           pulses_per_step=2, speed=speed, period_s=period_s)
    return result, bus.events


def test_slow_fixed_tick_move_uses_light_reads_and_ends_them():
    result, events = move(40, 2 / 150)
    assert result['reached']
    assert events[0] == 'light' and 'end' in events and not any(isinstance(e, tuple) for e in events)


def test_travel_move_still_streams():
    _, events = move(TRAVEL_GOAL_SPEED, 4 / 300)
    assert events[0] == ('stream', TRAVEL_GOAL_SPEED) and 'light' not in events


def test_old_untimed_slow_move_is_unchanged():
    _, events = move(40, None)                           # 고정 틱이 아닌 예전 느린 이동은 그대로(가벼운 읽기 안 씀)
    assert 'light' not in events
