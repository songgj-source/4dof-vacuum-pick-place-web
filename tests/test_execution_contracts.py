"""Cross-component HOME and unknown-object retry contracts; no device I/O."""
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'scripts')]
import arm_api
import sort_menu
from sorting.place_followup import run_home_return

HOME_OK = dict(home_return_complete=True, final_hold_confirmed=True, restoration_verified=True, error=None)


@pytest.mark.parametrize('code,changes', [
    (1, {}), (0, dict(final_hold_confirmed=False, final_hold_error='hold failed')),
    (0, dict(restoration_verified=False)), (0, dict(error='cleanup failed')),
])
def test_api_must_not_report_failed_home_as_success(monkeypatch, code, changes):
    summary = {**HOME_OK, **changes}
    arm = arm_api.Arm(run=lambda *a, **kw: (code, summary, 0.), read_state=lambda: 'ok', base_check=lambda: None)
    monkeypatch.setattr(arm_api.CAMERA, 'release', lambda: None)
    monkeypatch.setattr(sort_menu, 'live_view_running', lambda: False)
    assert not arm._execute('t', 'return_home', {})['ok']


def test_placed_recovery_must_not_ignore_home_exit_code(monkeypatch):
    arm = arm_api.Arm()
    replies = iter([(1, dict(ok=False, reason='automatic HOME failed'), 0.), (1, HOME_OK, 0.)])
    monkeypatch.setattr(arm, '_run', lambda *a: next(replies))
    monkeypatch.setattr(sort_menu, 'newest_result', lambda since: dict(motion_cycle_complete=True))
    assert not arm._pick('t', 'red_cube', 2)['ok']


def test_unknown_retry_without_saved_tray_really_drops_saved_frame(monkeypatch):
    arm = arm_api.Arm()
    calls = []
    def run(task, command):
        calls.append(command)
        if len(calls) == 1:
            return 1, dict(ok=False, reason='the tray moved since the stored frame'), 0.
        return 0, dict(ok=True), 0.
    monkeypatch.setattr(arm, '_run', run)
    monkeypatch.setattr(sort_menu.Path, 'exists', lambda self: True)
    assert arm._pick('t', 'unknown', 3)['ok']
    assert '--tray-frame' in calls[0]
    assert '--tray-frame' not in calls[1]


@pytest.mark.parametrize('suffix', ['[]', '123'])
def test_automatic_home_ignores_non_object_json_output(suffix):
    output = json.dumps(HOME_OK) + '\n' + suffix
    result = run_home_return(ROOT, 150, '/tmp/not-created', runner=lambda *a, **kw:
        SimpleNamespace(returncode=0, stdout=output, stderr=''))
    assert result['ok']


def test_automatic_home_fails_cleanly_without_json_summary():
    result = run_home_return(ROOT, 150, '/tmp/not-created', runner=lambda *a, **kw:
        SimpleNamespace(returncode=1, stdout='[]\n', stderr='device unavailable'))
    assert not result['ok']
    assert result['error'] == 'device unavailable'
