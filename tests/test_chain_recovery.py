"""Recovery from an infeasible tray-to-object path stays bounded and visible."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'scripts')]
import arm_api
import sort_menu
from sorting.sort_runner import run_sort


REFUSED = dict(ok=False, attempts=[dict(attempt=1, exit_code=4)], reason='pick_auto exited with 4; not retrying')


def test_runner_reports_home_recovery_instead_of_terminal_failure():
    result = run_sort(lambda: {}, lambda plan: 4)
    assert not result['ok']
    assert result['reason_code'] == 'chain_reset_required'
    assert result['attempts'] == [dict(attempt=1, exit_code=4)]
    assert 'HOME' in result['reason']


def test_expected_chain_reset_is_warning_but_other_failures_remain_errors():
    arm = arm_api.Arm()
    arm._log_run(['python', 'run_sort.py', '--start-above-tray'], 1, REFUSED)
    arm._log_run(['python', 'run_sort.py', '--start-above-tray'], 1,
                 dict(ok=False, attempts=[dict(exit_code=1)], reason='unsafe path'))
    rows, _ = arm.log.since(0)
    assert [row['level'] for row in rows] == ['warn', 'error']
    assert 'HOME' in rows[0]['message']


def run_recovery(monkeypatch, home_result, stop_after_home=False):
    monkeypatch.setattr(sort_menu, 'refresh_objects', lambda: None)
    calls, stages, stopped = [], [], [False]
    replies = iter([(0, dict(ok=True), 0.), (1, REFUSED, 0.), home_result])
    def run(command):
        calls.append(command)
        reply = next(replies)
        if 'return_home.py' in ' '.join(command) and stop_after_home:
            stopped[0] = True
        return reply
    outcome = sort_menu.run_all_chained(run=run, labels=['red_cube'],
                on_item=lambda label, stage: stages.append(stage), stopped=lambda: stopped[0])
    return outcome, calls, stages


def test_home_failure_reports_actual_recovery_error_without_second_home(monkeypatch):
    outcome, calls, stages = run_recovery(monkeypatch,
        (1, dict(home_return_complete=False, error='servo read timeout'), 0.))
    assert not outcome['ok']
    assert 'servo read timeout' in outcome['reason']
    assert len(calls) == 3
    assert 'recover_home' in stages


def test_stop_during_home_prevents_restarting_pick(monkeypatch):
    outcome, calls, _ = run_recovery(monkeypatch,
        (0, dict(home_return_complete=True, final_hold_confirmed=True, restoration_verified=True), 0.), stop_after_home=True)
    assert not outcome['ok']
    assert outcome['reason'] == 'cancelled'
    assert len(calls) == 3


def test_recovery_retries_same_object_from_home_once(monkeypatch):
    monkeypatch.setattr(sort_menu, 'refresh_objects', lambda: None)
    replies = iter([(0, dict(ok=True), 0.), (0, dict(ok=True), 0.),
                    (0, dict(ok=True), 0.), (1, REFUSED, 0.),
                    (0, dict(home_return_complete=True, final_hold_confirmed=True, restoration_verified=True), 0.), (0, dict(ok=True), 0.)])
    calls, stages = [], []
    def run(command):
        calls.append(command)
        return next(replies)
    outcome = sort_menu.run_all_chained(run=run, labels=['white_box', 'red_cube'],
                on_item=lambda label, stage: stages.append((label, stage)))
    assert outcome['ok']
    assert [item['object'] for item in outcome['items']] == ['white_box', 'red_cube']
    first, home, retry = calls[3:]
    assert '--start-above-tray' in first
    assert Path(home[1]).name == 'return_home.py'
    assert '--start-above-tray' not in retry
    assert retry[retry.index('--object') + 1] == 'red_cube'
    assert retry[retry.index('--plan-in') + 1] == first[first.index('--plan-in') + 1]
    assert stages[-2:] == [('red_cube', 'recover_home'), ('red_cube', 'retry_pick')]
