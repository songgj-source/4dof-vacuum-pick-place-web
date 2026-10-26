"""Exercise the script's stage boundary without opening hardware."""
import ast
from pathlib import Path
from types import SimpleNamespace

import pytest


def stage_environment(tmp_path, deviation=16):
    root = Path(__file__).resolve().parents[1]
    tree = ast.parse((root / 'scripts/return_home.py').read_text())
    stage = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'stage')
    snapshot = {'joints': {'wrist_flex': {'position': 3059}}}
    driver = SimpleNamespace(names=['wrist_flex'], hold_confirmed=True, _snapshot=lambda **kw: snapshot)
    def takeover(*args):
        raise ValueError('existing fixed goal is not stationary; takeover refused')
    env = dict(OUT=tmp_path, driver=driver, result={'phases': []},
               tcp=lambda s: (.079, 0., .109),
               home_arrival=lambda actual, expected: ('near' if deviation <= 32 else 'far', deviation),
               home_driver=takeover, bus=None, profile=None)
    import json
    import time
    env.update(json=json, time=time)
    exec(compile(ast.Module(body=[stage], type_ignores=[]), 'return_home_stage', 'exec'), env)
    return env, driver


def test_final_home_keeps_confirmed_driver_for_final_hold(tmp_path):
    env, driver = stage_environment(tmp_path)
    def endpoint_failure(record):
        raise ValueError('hover endpoint not reached')
    env['stage']('home', endpoint_failure, expected_raw={'wrist_flex': 3075})
    assert env['driver'] is driver
    assert env['driver'].hold_confirmed
    assert env['result']['home_deviation_pulses'] == 16
    assert env['result']['phases'][0]['observed']['joints']['wrist_flex']['position'] == 3059


def test_home_outside_arrival_tolerance_still_fails(tmp_path):
    env, _ = stage_environment(tmp_path, deviation=50)
    with pytest.raises(ValueError, match='HOME exceeds'):
        env['stage']('home', lambda record: {}, expected_raw={'wrist_flex': 3075})


def test_intermediate_stage_still_requires_takeover(tmp_path):
    env, driver = stage_environment(tmp_path)
    with pytest.raises(ValueError, match='existing fixed goal'):
        env['stage']('raise1', lambda record: {})
    assert env['driver'] is driver
