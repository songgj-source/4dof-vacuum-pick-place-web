"""Candidate HOME paths are sampled by the real executor, cancelled before writes."""
import ast
import json
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from motion.home_trial import run_recorded_home_trial
from motion.hover_trial import run_hover_trial
from motion.home_return_profile import return_profile
from robot.planar_kinematics import PlanarGeometry, NAMES


def test_recorded_tilted_pose_preflights_lifts_and_existing_fold_without_io():
    profile = json.loads((ROOT / 'config/servo_driver_candidate.json').read_text())
    joints = [SimpleNamespace(**j) for j in profile['joints']]
    zeros = {j.name: j.zero for j in joints}
    zeros.update(json.loads((ROOT / 'output/ik_resume_20261012/multipose_review/fit_full.json').read_text())['new_zero_pulses'])
    lengths = json.loads((ROOT / 'output/ik_resume_20261012/multipose_review/review.json').read_text())['eight_pose_geometry_m']
    geometry = PlanarGeometry(.027, .160, tuple(lengths))
    raw = dict(zip(NAMES, [2385, 3853, 1390, 2454]))
    def snapshot(**kw):
        return {'joints': {n: dict(position=v, goal=v) for n, v in raw.items()}}
    # No hardware class is instantiated; this bus has no I/O methods at all.
    driver = SimpleNamespace(names=NAMES, profile=SimpleNamespace(joints=joints),
                             state='hold_only', hold_confirmed=True, lock=threading.RLock(),
                             _snapshot=snapshot, clock=lambda: 0., bus=SimpleNamespace(),
                             _fault=lambda exc: None)
    angles = lambda values: tuple(-(values[n] - zeros[n]) * 2 * math.pi / 4096 for n in NAMES)
    pitch = geometry.forward(angles(raw))[3]
    phases = []
    def preflight(operation):
        events = []
        # Real executor checks every path sample before its first cancellation poll.
        with pytest.raises(InterruptedError, match='hover cancelled'):
            operation(events.append)
        prepared = next(e for e in events if e['phase'] == 'prepared')
        phases.append(prepared)
        raw.update(prepared['goal'])  # Model endpoint only, never a transmitted goal.
    for _ in range(3):
        xyz = geometry.forward(angles(raw))[:3]
        if xyz[2] >= .080:
            break
        target = (*xyz[:2], xyz[2] + .030)
        preflight(lambda record: run_hover_trial(driver, geometry, zeros, target,
                  math.hypot(*target[:2]), supported=True, lift_m=.030,
                  pitch_rad=pitch, record=record, cancelled=lambda: True))
    assert len(phases) == 2
    assert geometry.forward(angles(raw))[2] == pytest.approx(.10372234223656036)
    home = {n: j['position'] for n, j in json.loads((ROOT / 'output/ik_resume_20261012/original_current_hold.json').read_text())['joints'].items()}
    preflight(lambda record: run_recorded_home_trial(driver, geometry, zeros, home,
              supported=True, continuous=True, record=record, cancelled=lambda: True,
              **return_profile(300.)))
    assert raw == home
    assert phases[-1]['continuous_segments'] is True


@pytest.mark.parametrize('tray_start', [True, False])
def test_script_initial_lift_preserves_tilt_only_for_tray_route(tray_start):
    tree = ast.parse((ROOT / 'scripts/return_home.py').read_text())
    loop = next(n for n in ast.walk(tree) if isinstance(n, ast.For)
                and isinstance(n.target, ast.Name) and n.target.id == 'index')
    positions = iter([(.334, 0., .044), (.334, 0., .074), (.334, 0., .104)])
    calls = []
    driver = SimpleNamespace(_snapshot=lambda **kw: None)
    env = dict(folded=False, interrupted=False, driver=driver,
               tcp=lambda s: next(positions), tray_start=tray_start, start_pitch=-.85,
               g=None, zeros={}, math=math, stopped=lambda: False,
               run_hover_trial=lambda *a, **kw: calls.append(kw),
               stage=lambda name, operation, **kw: operation(lambda e: None))
    exec(compile(ast.Module(body=[loop], type_ignores=[]), 'home_lift', 'exec'), env)
    assert len(calls) == 2
    assert all(c['lift_m'] == .030 for c in calls)
    assert all(c['pitch_rad'] == (-.85 if tray_start else -math.pi/2) for c in calls)
