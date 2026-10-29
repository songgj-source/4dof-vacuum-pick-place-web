"""Guarded continuous rise after suction, with no hardware attached."""

import contextlib
import json
import math
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'scripts')]
from motion.hover_trial import run_hover_trial  # noqa: E402
from robot.planar_kinematics import NAMES  # noqa: E402


def dry_lift(via_z):
    pulse = 2 * math.pi / 4096
    joints = [SimpleNamespace(name=n, raw_min=0, raw_max=4095, direction=1) for n in NAMES]
    driver = SimpleNamespace(state='hold_only', hold_confirmed=True, names=NAMES,
                             lock=contextlib.nullcontext(), profile=SimpleNamespace(joints=joints),
                             _snapshot=lambda torque: {'joints': {n: {'position': 1000, 'goal': 1000} for n in NAMES}})
    geometry = SimpleNamespace(
        forward=lambda angles: (.22, .03, .018, -math.pi / 2),
        solve=lambda point, pitch, limits: [tuple((1000 + round(point[2] * 100)) * pulse for _ in NAMES)],
    )
    with patch('motion.hover_trial._execute_path', return_value={'reached': True}) as execute:
        result = run_hover_trial(
            driver, geometry, {n: 0 for n in NAMES}, (.22, .03, .070),
            math.hypot(.22, .03), supported=True, lift_m=.052,
            via=((.22, .03, via_z),), via_pulses=2,
            continuous_lift=True, pulses_per_step=2, quick_ticks=True, tick_s=2 / 150,
            min_contact_z=.010,
        )
    return result, execute


def test_post_pick_rise_is_one_stream_of_two_bounded_segments():
    result, execute = dry_lift(.044)
    assert result['reached']
    assert execute.call_count == 1
    assert execute.call_args.kwargs['continuous_segments'] is True


def test_post_pick_rise_rejects_a_segment_over_36_mm():
    with pytest.raises(ValueError, match='continuous lift segment'):
        dry_lift(.057)


def test_default_low_motion_rates_are_150_in_plan_only_output():
    completed = subprocess.run(
        [sys.executable, str(ROOT / 'scripts/pick_auto.py'),
         '--manual-target-mm', '223.07', '38.02', '17.5'],
        cwd=ROOT, capture_output=True, text=True, check=True,
    )
    result = json.loads(completed.stdout)
    assert result['motor_writes'] is False and result['pump_writes'] is False
    profile = result['plan']['motion_profile']
    assert profile['descent_rate_pulses_s'] == 600          # 2026-10-28 사용자 지시: 저속 동작 150 → 300 → 500 (접촉·탐침·재중심은 150 그대로)
    assert profile['held_rate_pulses_s'] == 600
    assert profile['slow_rate_pulses_s'] == 600


def test_place_pick_rises_to_70_mm_in_one_continuous_phase():
    import pick_auto

    calls = []
    contact = (.223, .038, .0175)
    start = (.222, .038, .018)
    phase = pick_auto.post_pick_rise(
        lambda name, target, **kwargs: calls.append((name, target, kwargs)),
        lambda: start, contact, placing=True,
    )

    assert phase == 'lift_to_clear'
    assert len(calls) == 1
    name, target, kwargs = calls[0]
    assert name == 'lift_to_clear'
    assert target == (.222, .038, .070)
    assert kwargs['continuous_lift'] is True
    assert kwargs['lift_m'] == pytest.approx(.052)
    assert kwargs['via'][0][2] == pytest.approx(.044)


def test_non_place_pick_keeps_small_lift_for_return():
    import pick_auto

    calls = []
    contact = (.223, .038, .0175)
    phase = pick_auto.post_pick_rise(
        lambda name, target, **kwargs: calls.append((name, target, kwargs)),
        lambda: (.223, .038, .018), contact, placing=False,
    )
    assert phase == 'lift20'
    assert len(calls) == 1 and calls[0][0] == 'lift20'
    assert calls[0][1] == pytest.approx((.223, .038, .0375))
    assert calls[0][2] == {'lift_m': .020}
