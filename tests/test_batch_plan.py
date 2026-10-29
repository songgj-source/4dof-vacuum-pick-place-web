"""책상 정리 계획을 물체마다 따로 하지 않고 한 번에 한다(사진·트레이·물체 검출 한 번, 2026-10-29). 하드웨어 접근 없음."""
import contextlib
import io
import json
import shutil
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'scripts'), str(ROOT / 'src')]
import plan_sort  # noqa: E402
import run_sort  # noqa: E402
import sort_menu  # noqa: E402

FRAME = ROOT / 'tests/data/classify_20261028'
TRAY = dict(found=True, rim_height_mm=40., method='walls', numbering='rule:far_from_base',
            slots={1: dict(x_mm=-150., y_mm=330., floor_height_mm=10., radius_m=.36), 2: dict(x_mm=-50., y_mm=330., floor_height_mm=10., radius_m=.33),
                   3: dict(x_mm=50., y_mm=330., floor_height_mm=10., radius_m=.34)})


def _plan(args):
    out = io.StringIO()
    with patch.object(plan_sort, 'detect_tray', return_value=TRAY), patch.object(plan_sort, 'detect_tray_checked', return_value=TRAY), contextlib.redirect_stdout(out):
        code = plan_sort.main(args + ['--from-files', str(FRAME / 'desk_rgb.png'), str(FRAME / 'desk_depth_mm.npy'), '--acknowledge-unverified-suction'])
    return code, out.getvalue()


@pytest.fixture
def clean_place_jobs():
    folder = ROOT / 'output/place_jobs'
    before = set(folder.glob('*')) if folder.exists() else set()
    yield
    for path in set(folder.glob('*')) - before:
        shutil.rmtree(path, ignore_errors=True)


def test_one_pass_plans_equal_one_by_one_plans_on_a_real_frame(clean_place_jobs):
    with tempfile.TemporaryDirectory() as directory:
        single = {}
        for label in ('red_cube', 'white_box'):
            code, text = _plan(['--object', label, '--output', str(Path(directory) / label)])
            assert code == 0
            single[label] = json.loads(text)
        code, text = _plan(['--objects', 'red_cube,white_box,tape_measure'])
    assert code == 0
    entries = {e['label']: e for e in json.loads(text)['plans']}
    for label in ('red_cube', 'white_box'):
        assert entries[label]['ok']
        one, many = single[label], entries[label]['plan']
        for key in ('slot_number', 'slot_source', 'manual_target_mm', 'camera_target_mm', 'warnings', 'tray_method', 'unregistered_left', 'tray_slots'):
            assert one[key] == many[key], (label, key)
        strip = lambda command: [c for c in command if 'place.json' not in str(c)]
        assert strip(one['pick_auto_command']) == strip(many['pick_auto_command'])
    assert not entries['tape_measure']['ok'] and 'not found in the current camera view' in entries['tape_measure']['reason']       # 없는 물체는 물체별로 실패


def test_objects_option_rejects_unknown_duplicates_and_mixing():
    for bad in (['--objects', 'unknown,red_cube'], ['--objects', 'red_cube,red_cube'], ['--objects', 'cup'], ['--objects', 'red_cube', '--object', 'white_box']):
        with pytest.raises(SystemExit) as exc:
            plan_sort.main(bad + ['--from-files', str(FRAME / 'desk_rgb.png'), str(FRAME / 'desk_depth_mm.npy')])
        assert exc.value.code == 2


def test_run_sort_writes_one_plan_file_per_object_and_reports_missing_ones(tmp_path):
    entries = [dict(label='red_cube', ok=True, plan=dict(label='red_cube', unregistered_left=1, pick_auto_command=['x'])),
               dict(label='white_box', ok=False, reason='white_box not found in the current camera view')]
    done = SimpleNamespace(returncode=0, stdout=json.dumps(dict(plans=entries)), stderr='')
    printed = []
    with patch.object(run_sort.subprocess, 'run', return_value=done) as camera, patch('builtins.print', side_effect=lambda *a, **k: printed.append(a[0])):
        assert run_sort.main(['--objects', 'red_cube,white_box', '--plan-out-dir', str(tmp_path), '--plan-prefix', 'T_']) == 0
    camera.assert_called_once()                                            # 프로그램·카메라는 한 번
    result = json.loads(printed[-1])
    assert result['ok'] and result['plans']['red_cube'] == dict(ok=True, plan_file=str(tmp_path / 'T_red_cube.json'), unregistered_left=1)
    assert json.loads((tmp_path / 'T_red_cube.json').read_text())['pick_auto_command'] == ['x']
    assert not result['plans']['white_box']['ok'] and 'not found in the current camera view' in result['plans']['white_box']['reason']
    failed = SimpleNamespace(returncode=2, stdout='', stderr='plan_sort.py: error: tray not found')
    with patch.object(run_sort.subprocess, 'run', return_value=failed), patch('builtins.print', side_effect=lambda *a, **k: printed.append(a[0])):
        assert run_sort.main(['--objects', 'red_cube,white_box', '--plan-out-dir', str(tmp_path)]) == 1
    assert 'planning failed' in json.loads(printed[-1])['reason']


def _fake_run(batch_result=None, per_label=None):
    calls = []

    def run(command):
        calls.append([str(c) for c in command])
        text = ' '.join(calls[-1])
        if '--objects' in command:
            return batch_result
        if '--plan-out' in text:
            return per_label[command[command.index('--object') + 1]]
        return 0, dict(ok=True), 1.
    return run, calls


def _batch(tmp_path, **statuses):
    plans = {}
    for label, status in statuses.items():
        plans[label] = dict(ok=True, plan_file=str(tmp_path / f'{label}.json'), unregistered_left=0) if status else \
            dict(ok=False, reason=f'planning failed: plan_sort.py: error: {label} not found in the current camera view')
    return 0, dict(ok=True, plans=plans), 1.


def test_chain_plans_everything_in_one_command_then_picks_with_those_files(tmp_path):
    run, calls = _fake_run(batch_result=_batch(tmp_path, red_cube=True, white_box=False, tape_measure=True))
    outcome = sort_menu.run_all_chained(run=run, batch_plan=True)
    assert outcome['ok'] and outcome['skipped'] == ['white_box']
    plan_calls = [c for c in calls if '--plan-out' in c or '--objects' in c]
    assert len(plan_calls) == 1 and 'red_cube,white_box,tape_measure' in plan_calls[0]                 # 계획 명령은 한 번
    picks = [c for c in calls if '--plan-in' in c]
    assert [c[c.index('--plan-in') + 1] for c in picks] == [str(tmp_path / 'red_cube.json'), str(tmp_path / 'tape_measure.json')]


def test_chain_falls_back_to_per_object_planning_when_the_batch_fails(tmp_path):
    per = {name: (0, dict(ok=True), 1.) for name in ('red_cube', 'white_box', 'tape_measure')}
    run, calls = _fake_run(batch_result=(1, dict(ok=False, reason='planning failed: tray not found'), 1.), per_label=per)
    outcome = sort_menu.run_all_chained(run=run, batch_plan=True)
    assert outcome['ok']
    assert sum('--plan-out' in c for c in calls) == 3                                                   # 예전처럼 물체마다(한 번에 계획한 명령은 --plan-out-dir)


def test_chain_stops_at_the_first_real_planning_failure_like_before(tmp_path):
    batch = (0, dict(ok=True, plans=dict(red_cube=dict(ok=True, plan_file=str(tmp_path / 'r.json'), unregistered_left=0),
                                         white_box=dict(ok=False, reason='planning failed: plan_sort.py: error: tray not found'),
                                         tape_measure=dict(ok=True, plan_file=str(tmp_path / 't.json'), unregistered_left=0))), 1.)
    run, calls = _fake_run(batch_result=batch)
    outcome = sort_menu.run_all_chained(run=run, batch_plan=True)
    assert not outcome['ok'] and outcome['failed_label'] == 'white_box' and 'tray not found' in outcome['reason']
    picks = [c for c in calls if '--plan-in' in c]                                                      # 앞에서 계획된 물체는 넣고 실패한 물체에서 멈춘다(예전과 같다)
    assert [c[c.index('--plan-in') + 1] for c in picks] == [str(tmp_path / 'r.json')]


def test_server_turns_batch_planning_on_by_default_and_the_env_can_disable_it():
    source = (ROOT / 'scripts/arm_api.py').read_text()
    assert "batch_plan=os.environ.get('ARM_BATCH_PLAN', '1') != '0'" in source
