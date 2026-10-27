"""One checked stream from suction contact to the tilted carry pose."""
import contextlib
import ast
import math
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from robot.planar_kinematics import NAMES
from motion.pick_to_carry import build_lift_transition, run_pick_to_carry_trial, LiftPathRefused

START = (.255, -.012, .0175)
PLAN = dict(start=(*START[:2], .070), executed=False, usable_for_motion=False, legs=[
    dict(name='raise_clear', target=(*START[:2], .070)),
    dict(name='raise', target=(*START[:2], .100)),
    dict(name='tilt', target=(*START[:2], .100), pitch_rad=-math.pi / 4),
    dict(name='carry', target=(.3076, -.1474, .100), pitch_rad=-math.pi / 4),
    dict(name='lower', target=(.3076, -.1474, .060), pitch_rad=-math.pi / 4),
])


def test_low_section_stays_vertical_then_rise_and_tilt_share_waypoints():
    result = build_lift_transition(PLAN, START)
    assert result['target'] == (*START[:2], .100)
    low = [p for p in result['waypoints'] if p['xyz'][2] <= .070]
    assert low and low[-1]['xyz'][2] == .070
    assert all(p['pitch_rad'] == -math.pi / 2 for p in low)
    high = [p for p in result['waypoints'] if p['xyz'][2] > .070]
    assert len(high) >= 2
    assert high[-1]['pitch_rad'] == -math.pi / 4
    assert all(a['xyz'][2] < b['xyz'][2] and a['pitch_rad'] < b['pitch_rad'] for a, b in zip(high, high[1:]))
    assert result['remaining_plan']['start'] == (*START[:2], .100)
    assert [leg['name'] for leg in result['remaining_plan']['legs']] == ['carry', 'lower']
    assert [leg['name'] for leg in PLAN['legs']] == ['raise_clear', 'raise', 'tilt', 'carry', 'lower']


def test_pull_in_path_and_already_tilted_pick_keep_existing_sequence():
    plan = {**PLAN, 'legs': [dict(name='pull_in', target=(.24, -.012, .07)), *PLAN['legs']]}
    assert build_lift_transition(plan, START) is None
    assert build_lift_transition(PLAN, START, start_pitch=-math.pi / 2 + .1) is None


def test_held_object_clearance_can_delay_tilt_without_an_intermediate_hold():
    transition = build_lift_transition(PLAN, START, tilt_start_z=.085)
    assert transition['tilt_start_z'] == .085
    assert all(p['pitch_rad'] == -math.pi / 2 for p in transition['waypoints'] if p['xyz'][2] <= .085)
    assert transition['waypoints'][-1]['pitch_rad'] == -math.pi / 4
    assert build_lift_transition(PLAN, START, tilt_start_z=.100) is None


def fixtures():
    pulse = 2 * math.pi / 4096
    joints = [SimpleNamespace(name=n, raw_min=0, raw_max=4095, direction=1) for n in NAMES]
    snapshot = dict(joints={n:dict(position=1000, goal=1000) for n in NAMES})
    driver = SimpleNamespace(state='hold_only', hold_confirmed=True, names=NAMES,
                lock=contextlib.nullcontext(), profile=SimpleNamespace(joints=joints),
                _snapshot=lambda **kw: snapshot)
    def forward(q):
        return (*START[:2], START[2] + (q[0] / pulse - 1000) * .00025,
                -math.pi / 2 + (q[1] / pulse - 1000) * .005)
    def solve(point, pitch, limits):
        return [(pulse * (1000 + (point[2] - START[2]) / .00025),
                 pulse * (1000 + (pitch + math.pi / 2) / .005), 1000 * pulse, 1000 * pulse)]
    geometry = SimpleNamespace(forward=forward, solve=solve,
                link_points=lambda q: [(0., 0., .16), forward(q)[:3]])
    return driver, geometry, dict.fromkeys(NAMES, 0)


def test_merged_lift_uses_one_existing_executor_call_without_midpoint_hold():
    driver, geometry, zeros = fixtures()
    transition = build_lift_transition(PLAN, START)
    with patch('motion.pick_to_carry._execute_path', return_value={'reached': True}) as execute:
        result = run_pick_to_carry_trial(driver, geometry, zeros, transition, supported=True, rate=150, min_contact_z=.010)
    assert result['reached']
    assert execute.call_count == 1
    assert len(execute.call_args.kwargs['waypoints']) == len(transition['waypoints'])
    assert execute.call_args.kwargs['period_s'] == pytest.approx(2 / 150)
    assert execute.call_args.kwargs['pulses_per_step'] == 2


def test_unreachable_transition_is_refused_before_execution():
    driver, geometry, zeros = fixtures()
    geometry.solve = lambda *a: []
    with patch('motion.pick_to_carry._execute_path') as execute:
        with pytest.raises(LiftPathRefused):
            run_pick_to_carry_trial(driver, geometry, zeros, build_lift_transition(PLAN, START), supported=True)
    execute.assert_not_called()


def test_low_tilt_is_refused_before_execution():
    driver, geometry, zeros = fixtures()
    transition = build_lift_transition(PLAN, START)
    transition['waypoints'][0]['pitch_rad'] += .2
    with patch('motion.pick_to_carry._execute_path') as execute:
        with pytest.raises(LiftPathRefused):
            run_pick_to_carry_trial(driver, geometry, zeros, transition, supported=True)
    execute.assert_not_called()


def test_bowed_joint_interpolation_cannot_start_tilting_below_70mm():
    driver, geometry, zeros = fixtures()
    original = geometry.forward
    def bowed(q):
        x, y, z, pitch = original(q)
        if math.radians(2.5) < pitch - VERTICAL < math.radians(4):
            z = .0685
        return x, y, z, pitch
    VERTICAL = -math.pi / 2
    geometry.forward = bowed
    geometry.link_points = lambda q: [(0., 0., .16), bowed(q)[:3]]
    with patch('motion.pick_to_carry._execute_path') as execute:
        with pytest.raises(LiftPathRefused, match='tilt below clearance'):
            run_pick_to_carry_trial(driver, geometry, zeros, build_lift_transition(PLAN, START), supported=True)
    execute.assert_not_called()


def test_resumed_place_starts_with_carry_and_preserves_tilt():
    from sorting.place_exec import run_place
    transition = build_lift_transition(PLAN, START)
    pose, moves, releases = [transition['target']], [], []
    def hover(name, target, **kw):
        moves.append((name, kw))
        pose[0] = tuple(target)
    report = run_place(transition['remaining_plan'], hover=hover, fresh_xyz=lambda: pose[0],
                       release=lambda point: releases.append(point), cancelled=lambda: False)
    assert report['released'] and len(releases) == 1
    assert moves[0][0].startswith('place_carry')
    assert all(not name.startswith(('place_raise', 'place_tilt')) for name, _ in moves)
    assert all(kw['pitch_rad'] == -math.pi / 4 for _, kw in moves)


@pytest.mark.parametrize('error,retries', [(LiftPathRefused('unreachable'), True),
                                         (ValueError('hover tracking error'), False),
                                         (InterruptedError('cancelled'), False)])
def test_only_preparation_refusal_can_use_legacy_lift(error, retries):
    # Execute the real orchestration branch with I/O replaced at its boundary.
    tree = ast.parse((ROOT / 'scripts/pick_auto.py').read_text())
    merge = next(n for n in ast.walk(tree) if isinstance(n, ast.If) and ast.unparse(n.test) == 'transition is not None')
    fallback = next(n for n in ast.walk(tree) if isinstance(n, ast.If) and ast.unparse(n.test) == 'lift_place_plan is None')
    calls = []
    def trial(*a, **kw):
        raise error
    env = dict(transition=build_lift_transition(PLAN, START), result={}, lift_place_plan=None,
        LiftPathRefused=LiftPathRefused, stage=lambda name, target, operation: operation(lambda e: None),
        run_pick_to_carry_trial=trial, driver=None, geometry=None, zeros={},
        args=SimpleNamespace(held_rate=150, min_contact_z_mm=10), is_cancelled=lambda: False,
        post_pick_rise=lambda *a, **kw: calls.append('legacy'), hover=None, fresh_xyz=None,
        contact=START, place_job=PLAN, lift_plan=PLAN)
    code = compile(ast.Module(body=[merge, fallback], type_ignores=[]), 'lift_orchestration', 'exec')
    if retries:
        exec(code, env)
        assert calls == ['legacy']
    else:
        with pytest.raises(type(error), match=str(error)):
            exec(code, env)
        assert not calls


def test_saved_pickup_pose_passes_real_geometry_full_path_preflight_without_io():
    from robot.robot_model import load_planar_model
    from hardware.servo_mapping import MappingProfile
    model = load_planar_model(ROOT)
    profile = MappingProfile.load(ROOT / 'config/servo_driver_candidate.json', ROOT)
    joints = {j.name: j for j in profile.joints}
    raw = dict(zip(NAMES, [2131, 3749, 1598, 2828]))
    q = [joints[n].direction * (raw[n] - model['zeros'][n]) * 2 * math.pi / 4096 for n in NAMES]
    start = model['geometry'].forward(q)[:3]
    legs = [{**leg, 'target': (*start[:2], leg['target'][2])} if leg['name'] != 'carry' and leg['name'] != 'lower' else dict(leg)
            for leg in PLAN['legs']]
    transition = build_lift_transition({**PLAN, 'start': (*start[:2], .070), 'legs': legs}, start, tilt_start_z=.0705)
    driver = SimpleNamespace(state='hold_only', hold_confirmed=True, names=NAMES,
                lock=contextlib.nullcontext(), profile=profile,
                _snapshot=lambda **kw: dict(joints={n: dict(position=v, goal=v) for n, v in raw.items()}))
    events = []
    with patch('motion.pick_to_carry._execute_path', return_value={'prepared_only': True}) as execute:
        result = run_pick_to_carry_trial(driver, model['geometry'], model['zeros'], transition,
                                         supported=True, min_contact_z=.010, record=events.append)
    assert result['prepared_only']
    assert execute.call_count == 1
    assert events[-1]['points'] == 5
    assert events[-1]['intermediate_holds'] == 0


def test_side_drift_is_tight_near_the_desk_and_looser_once_lifted():
    # 2026-10-27 Pi: 정육면체를 41mm 들고 기울이는 중 늦게 따라온 관절 때문에 컵이 4.6mm 옆으로 가 멈췄다(3mm 한도)
    from robot.robot_model import load_planar_model
    from hardware.servo_mapping import MappingProfile
    model = load_planar_model(ROOT)
    profile = MappingProfile.load(ROOT / 'config/servo_driver_candidate.json', ROOT)
    joints = {j.name: j for j in profile.joints}
    raw = dict(zip(NAMES, [2131, 3749, 1598, 2828]))
    pulse = 2 * math.pi / 4096
    q0 = [joints[n].direction * (raw[n] - model['zeros'][n]) * pulse for n in NAMES]
    geometry = model['geometry']
    start = geometry.forward(q0)[:3]
    legs = [{**leg, 'target': (*start[:2], leg['target'][2])} if leg['name'] not in ('carry', 'lower') else dict(leg) for leg in PLAN['legs']]
    transition = build_lift_transition({**PLAN, 'start': (*start[:2], .070), 'legs': legs}, start, tilt_start_z=.0705)
    driver = SimpleNamespace(state='hold_only', hold_confirmed=True, names=NAMES, lock=contextlib.nullcontext(), profile=profile,
                             _snapshot=lambda **kw: dict(joints={n: dict(position=v, goal=v) for n, v in raw.items()}))
    captured = {}
    with patch('motion.pick_to_carry._execute_path', side_effect=lambda *a, **k: captured.update(args=a) or {}):
        run_pick_to_carry_trial(driver, geometry, model['zeros'], transition, supported=True, min_contact_z=.010)
    clear = captured['args'][4]
    limits = {n: tuple(sorted(joints[n].direction * (r - model['zeros'][n]) * pulse for r in (joints[n].raw_min + 16, joints[n].raw_max - 16))) for n in NAMES}
    radial = (start[0] / math.hypot(*start[:2]), start[1] / math.hypot(*start[:2]))

    def raw_at(dz, side):
        q = min(geometry.solve((start[0] + radial[0] * side, start[1] + radial[1] * side, start[2] + dz), -math.pi / 2, limits),
                key=lambda q: sum((a - b) ** 2 for a, b in zip(q, q0)))
        return {n: round(model['zeros'][n] + q[i] / (joints[n].direction * pulse)) for i, n in enumerate(NAMES)}
    with pytest.raises(ValueError, match='must stay above the pickup point'):
        clear(raw_at(.010, .005))                         # 아직 책상 가까이(10mm): 5mm 옆은 멈춘다
    clear(raw_at(.040, .005))                             # 40mm 든 뒤: 5mm 옆은 계속 간다
    with pytest.raises(ValueError, match='must stay above the pickup point'):
        clear(raw_at(.040, .010))                         # 그래도 10mm는 멈춘다(가드 배율 1)
