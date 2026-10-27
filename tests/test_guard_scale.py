"""가드 배율(config/guard_scale.json, 사용자 지시 2026-10-26 1.5배): 판정 기준만 넓히고 넓히지 않을 것은 그대로인지."""
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'scripts')]
from motion import guard_mode  # noqa: E402
from motion.home_return_profile import home_arrival  # noqa: E402
from motion.trial_phase import phase_pose_tolerance, approach_correction_target  # noqa: E402


def test_production_config_is_one_and_a_half(monkeypatch):
    monkeypatch.delenv('VACUUM_GUARD_SCALE', raising=False)
    assert json.loads((ROOT / 'config/guard_scale.json').read_text())['scale'] == 1.5
    assert guard_mode.scale() == 1.5
    assert (guard_mode.pulses(2), guard_mode.pulses(1), guard_mode.pulses(8), guard_mode.pulses(1600)) == (3, 1, 12, 2400)


def test_strict_mode_and_test_default_keep_the_old_values(monkeypatch):
    assert guard_mode.scale() == 1.0                                   # tests/conftest.py
    monkeypatch.setenv('VACUUM_GUARD_SCALE', '1.5')
    monkeypatch.setenv('VACUUM_STRICT_GUARDS', '1')
    assert guard_mode.scale() == 1.0 and guard_mode.pulses(8) == 8


@pytest.mark.parametrize('value', ['0.5', '3', 'nan'])
def test_scale_outside_range_is_refused(monkeypatch, value):
    monkeypatch.setenv('VACUUM_GUARD_SCALE', value)
    with pytest.raises(ValueError):
        guard_mode.scale()


def test_judgement_thresholds_follow_the_scale(monkeypatch):
    base = {name: phase_pose_tolerance(name) for name in ('approach', 'place_lower_0', 'lift_to_carry', 'place_carry_0')}
    monkeypatch.setenv('VACUUM_GUARD_SCALE', '1.5')
    assert {k: round(phase_pose_tolerance(k), 6) for k in base} == {k: round(v * 1.5, 6) for k, v in base.items()}
    target = (.25, 0., .07)
    assert approach_correction_target(target, (.25 - .025, 0., .07))   # 25mm: 1.5배(30mm) 안
    monkeypatch.setenv('VACUUM_GUARD_SCALE', '1')
    with pytest.raises(ValueError, match='20mm'):
        approach_correction_target(target, (.25 - .025, 0., .07))
    home = {'a': 2000}
    assert home_arrival({'a': 2011}, home)[0] == 'near'               # 배율 1: 8펄스 넘으면 exact가 아니다
    monkeypatch.setenv('VACUUM_GUARD_SCALE', '1.5')
    assert home_arrival({'a': 2011}, home)[0] == 'exact'              # 1.5배: 12펄스까지 exact
    assert home_arrival({'a': 2061}, home)[0] == 'far'                # 40×1.5=60 넘으면 실패는 그대로


def test_collision_and_limit_constants_are_not_scaled(monkeypatch):
    monkeypatch.setenv('VACUUM_GUARD_SCALE', '1.5')
    from sorting import place_plan
    from motion import hover_trial
    assert place_plan.OBSTACLE_MARGIN_MM == 10. and place_plan.CARRY_Z == .130
    assert hover_trial.LIFT_MAX_M == .036                              # 계획 값은 그대로(받아들이는 쪽만 넓어진다)


def test_pick_to_carry_stay_above_check_follows_the_scale():
    src = (ROOT / 'src/motion/pick_to_carry.py').read_text()
    assert 'math.hypot(xyz_pitch[0] - start[0], xyz_pitch[1] - start[1]) > guard_mode.length(allowed)' in src   # 실제 위치 검사(3mm, 뜬 뒤 8mm)
    assert 'LIFT_XY_TIGHT_M, LIFT_XY_FREE_M, LIFT_FREE_ABOVE_M = .003, .008, .020' in src
    assert "for leg in prefix):" in src and "> .003 for leg in prefix" in src                                   # 계획 경로 검사는 그대로


def test_pick_to_carry_servo_speed_follows_the_path_rate():
    # 서보 속도 레지스터 40은 관절을 초당 약 40펄스에 묶어 물체를 든 어깨가 경로를 못 따라갔다(2026-10-26 Pi)
    src = (ROOT / 'src/motion/pick_to_carry.py').read_text()
    assert 'servo_speed = max(40, int(math.ceil(rate)))' in src and 'speed=servo_speed, period_s=2 / rate' in src
