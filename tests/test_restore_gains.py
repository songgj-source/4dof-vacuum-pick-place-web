"""임시 게인이 서보에 남았을 때(2026-10-28 return_home 중간 종료) 기준값으로 되돌린다. 기본은 읽기만."""
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path[:0] = [str(Path(__file__).resolve().parents[1] / 'src')]
from motion.trial_gains import BASELINE, restore_baseline_gains

ADDR = dict(p=21, d=22, i=23, startup=24, lock=55)


class FakeBus:
    def __init__(self, rows):
        self.regs = {m: {ADDR[k]: v for k, v in row.items()} for m, row in rows.items()}
        self.writes = []
        self.port = SimpleNamespace(ser=SimpleNamespace(reset_input_buffer=lambda: None))
        self.packet = SimpleNamespace(write1ByteTxRx=self._write)

    def _write(self, _port, motor, address, value):
        self.writes.append((motor, address, value))
        self.regs[motor][address] = value
        return (0, 0, 0)

    def _read_register(self, motor, address, size):
        return self.regs[motor][address]


def _stuck():
    return {1: dict(BASELINE), 2: dict(BASELINE, p=48, i=1), 3: dict(BASELINE, p=48)}        # 실제로 남았던 값


def test_dry_run_only_reads(tmp_path):
    bus = FakeBus(_stuck())
    report = restore_baseline_gains(bus, tmp_path, sleep=lambda s: None)
    assert bus.writes == []
    assert [report[m]['baseline_ok'] for m in '123'] == [True, False, False]


def test_execute_restores_only_p_d_i_that_differ_and_verifies(tmp_path):
    bus = FakeBus(_stuck())
    report = restore_baseline_gains(bus, tmp_path, execute=True, sleep=lambda s: None)
    assert all(row['verified'] for row in report.values())
    assert bus.regs[2] == {ADDR[k]: v for k, v in BASELINE.items()} and bus.regs[3] == {ADDR[k]: v for k, v in BASELINE.items()}
    assert not any(motor == 1 for motor, _, _ in bus.writes)                                  # 이미 기준인 베이스는 건드리지 않는다
    assert [w for w in bus.writes if w[0] == 3] == [(3, 55, 0), (3, 21, 16), (3, 55, 1)]       # 잠금 해제 → P → 잠금
    assert [w for w in bus.writes if w[0] == 2] == [(2, 55, 0), (2, 21, 16), (2, 23, 0), (2, 55, 1)]


def test_startup_mismatch_is_reported_not_changed(tmp_path):
    bus = FakeBus({2: dict(BASELINE, startup=99)})
    report = restore_baseline_gains(bus, tmp_path, motors=(2,), execute=True, sleep=lambda s: None)
    assert bus.writes == [] and not report['2']['baseline_ok']


def test_already_baseline_writes_nothing(tmp_path):
    bus = FakeBus({m: dict(BASELINE) for m in (1, 2, 3)})
    report = restore_baseline_gains(bus, tmp_path, execute=True, sleep=lambda s: None)
    assert bus.writes == [] and all(row['verified'] and not row['changed'] for row in report.values())
