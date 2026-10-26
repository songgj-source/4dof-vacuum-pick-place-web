"""Run the real HOME start selection without transport or motor access."""
import ast
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from motion.home_return_profile import home_arrival, near_fold
from motion.return_corridor import interrupted_near_home_start, start_in_corridor


@pytest.mark.parametrize('admit_tray', [False, True])
def test_corridor_selection_keeps_the_actual_start_snapshot(tmp_path, admit_tray):
    root = Path(__file__).resolve().parents[1]
    tree = ast.parse((root / 'scripts/return_home.py').read_text())
    block = next(n for n in ast.walk(tree) if isinstance(n, ast.With)
                 and any(isinstance(i.context_expr, ast.Name) and i.context_expr.id == 'gains'
                         for i in n.items))
    start = next(i for i, n in enumerate(block.body) if isinstance(n, ast.Assign)
                 and any(isinstance(t, ast.Name) and t.id == 'initial' for t in n.targets))
    end = next(i for i, n in enumerate(block.body) if isinstance(n, ast.FunctionDef) and n.name == 'stage')
    home_path = tmp_path / 'output/ik_resume_20261012/original_current_hold.json'
    home_path.parent.mkdir(parents=True)
    home = dict(base=2110, shoulder=1823, elbow=3018, wrist_flex=3075)
    home_path.write_text(json.dumps({'joints': {n: {'position': v} for n, v in home.items()}}))
    raw = dict(base=2385, shoulder=3853, elbow=1390, wrist_flex=2454)
    snapshot = {'joints': {n: {'position': v} for n, v in raw.items()}}
    xyz = (.3037794671224337, -.1397053796445158, .04365193813923507) if admit_tray else (.356, 0., .04365)
    pitch = -.8498585976343079
    result = {}
    env = dict(ROOT=tmp_path, json=json, driver=SimpleNamespace(names=tuple(raw),
               _snapshot=lambda **kw: snapshot), tcp=lambda s: (*xyz, pitch), result=result,
               home_arrival=home_arrival, near_fold=near_fold,
               interrupted_near_home_start=interrupted_near_home_start,
               start_in_corridor=start_in_corridor, args=SimpleNamespace(any_start=False, travel_rate=300.),
               travel={}, recorder=None, HOVER_RADIUS_MAX_M=.355, MAX_TILT_RAD=math.radians(60))
    # Include any new pure diagnostic imports added to the script.
    import motion.return_corridor as corridor
    env.update(vars(corridor))
    if admit_tray:
        exec(compile(ast.Module(body=block.body[start:end], type_ignores=[]), 'home_start', 'exec'), env)
        assert result['tilted_tray_start'] is True
        assert not env['folded'] and not env['interrupted']
    else:
        with pytest.raises(ValueError, match='outside the vertical reset corridor: r=0.356000 m, z=0.043650 m, pitch=-48.693 deg'):
            exec(compile(ast.Module(body=block.body[start:end], type_ignores=[]), 'home_start', 'exec'), env)
    assert result['start_snapshot'] is snapshot
    assert result['start_candidate_xyz'] == xyz
    assert result['start_raw'] == raw
    assert result['start_candidate_pitch_rad'] == pitch
    diagnostic = result['start_corridor']
    assert diagnostic['accepted'] is False
    assert diagnostic['radius_m'] == pytest.approx(math.hypot(*xyz[:2]))
    assert diagnostic['violations'] == ['radius_above_max']
    assert diagnostic['radius_bounds_m'] == [.14, .32]
    assert diagnostic['height_bounds_m'] == [.030, .135]
