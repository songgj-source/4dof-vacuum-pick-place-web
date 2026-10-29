"""정지하면 그 자리에 서고, '다시 시작해'로 하던 작업을 이어서 한다(먼저 HOME 복귀, 정리는 남은 물체만)."""
import sys
from pathlib import Path
from unittest.mock import Mock, patch

sys.path[:0] = [str(Path(__file__).resolve().parents[1] / 'scripts'), str(Path(__file__).resolve().parents[1] / 'src')]
import arm_api
import arm_llm

HOME_OK = dict(home_return_complete=True, final_hold_confirmed=True, restoration_verified=True)
CANCELLED = dict(ok=False, reason='cancelled', reason_code='cancelled')


def _arm(run=None):
    arm = arm_api.Arm(run=run or Mock(return_value=(0, dict(HOME_OK), 1.)), read_state=lambda: 'ok', base_check=lambda: None)
    arm.tasks['t'] = dict(id='t', skill='x', params={}, state='running', phase=None, phase_label=None, phases=[], warnings=[],
                          result=None, item=None, items=[], started=0., seconds=0.)
    arm.current = 't'
    return arm


def _record(arm, skill, params, result=None, items=()):
    arm.tasks['t']['items'] = list(items)
    arm._record_resume('t', skill, params, result or dict(CANCELLED))
    return arm.resume


def test_cancelled_pick_place_is_remembered_with_same_object_and_slot():
    assert _record(_arm(), 'pick_place', dict(object='red_cube', slot=3)) == dict(skill='pick_place', params=dict(object='red_cube', slot=3))


def test_only_a_stop_is_resumable_not_failures_or_successes_or_other_skills():
    for result in (dict(ok=False, reason='x', reason_code='failed'), dict(ok=True, reason=None, reason_code=None)):
        assert _record(_arm(), 'pick_place', dict(object='red_cube', slot=1), result) is None
    assert _record(_arm(), 'return_home', {}) is None


def test_pick_all_keeps_only_the_objects_not_yet_placed():
    items = [dict(object='red_cube', ok=True), dict(object='white_box', ok=False)]
    state = _record(_arm(), 'pick_all', dict(objects=['red_cube', 'white_box', 'tape_measure']), items=items)
    assert state['params'] == dict(objects=['white_box', 'tape_measure'])                 # 실패·미완은 다시, 끝낸 것은 뺀다


def test_pick_all_without_object_list_after_some_done_becomes_an_explicit_remaining_list():
    state = _record(_arm(), 'pick_all', {}, items=[dict(object='red_cube', ok=True)])
    assert state['params'] == dict(objects=['white_box', 'tape_measure'])
    assert _record(_arm(), 'pick_all', {}) ['params'] == {}                              # 하나도 못 했으면 원래대로(전부)


def test_pick_all_keeps_include_new_and_forgets_when_nothing_is_left():
    items = [dict(object=o, ok=True) for o in ('red_cube', 'white_box', 'tape_measure')]
    assert _record(_arm(), 'pick_all', {}, items=items) is None
    state = _record(_arm(), 'pick_all', dict(objects=['red_cube', 'white_box', 'tape_measure'], include_new=True), items=items)
    assert state['params'] == dict(objects=[], include_new=True)                          # 새 물체만 남았다


def test_resume_goes_home_first_then_runs_the_rest():
    run = Mock(return_value=(0, dict(HOME_OK), 1.))
    arm = _arm(run)
    arm.resume = dict(skill='pick_all', params=dict(objects=['white_box', 'tape_measure']))
    arm._execute = Mock(return_value=dict(ok=True, reason=None, reason_code=None))
    with patch.object(arm_api.CAMERA, 'release'):
        result = arm._resume('t')
    command = run.call_args.args[0]
    assert Path(command[1]).name == 'return_home.py' and '--any-start' in command
    arm._execute.assert_called_once_with('t', 'pick_all', dict(objects=['white_box', 'tape_measure']))
    assert result['ok'] and result['resumed'] and arm.resume is None


def test_resume_drive_does_not_touch_the_arm():
    run = Mock()
    arm = _arm(run)
    arm.resume = dict(skill='drive', params=dict(label='책상', mode='precise', then='tidy'))
    arm._execute = Mock(return_value=dict(ok=True, reason=None, reason_code=None))
    arm._resume('t')
    run.assert_not_called()
    arm._execute.assert_called_once_with('t', 'drive', dict(label='책상', mode='precise', then='tidy'))


def test_failed_or_cancelled_home_keeps_the_task_for_another_try():
    for code, summary, expect in ((1, dict(error='clearance'), 'failed'), (130, dict(ok=False, reason='cancelled'), 'cancelled')):
        arm = _arm(Mock(return_value=(code, summary, 1.)))
        state = dict(skill='pick_place', params=dict(object='white_box', slot=2))
        arm.resume = state
        arm._execute = Mock()
        with patch.object(arm_api.CAMERA, 'release'):
            result = arm._resume('t')
        assert not result['ok'] and arm.resume == state and result['reason_code'] == expect
        arm._execute.assert_not_called()


def test_stopping_again_while_resuming_remembers_what_is_left():
    arm = _arm()
    arm.resume = dict(skill='pick_all', params=dict(objects=['white_box', 'tape_measure']))
    def inner(task_id, skill, params):
        arm.tasks['t']['items'] = [dict(object='white_box', ok=True)]
        return dict(CANCELLED)
    arm._execute = inner
    with patch.object(arm_api.CAMERA, 'release'):
        arm._resume('t')
    assert arm.resume == dict(skill='pick_all', params=dict(objects=['tape_measure']))


def test_resume_needs_something_to_resume_and_a_new_task_forgets_it():
    arm = arm_api.Arm()
    status, body = arm.start('resume', {})
    assert status == 409 and body['reason_code'] == 'nothing_to_resume'
    arm.resume = dict(skill='pick_place', params=dict(object='red_cube', slot=1))
    with patch.object(arm_api.threading, 'Thread'):
        status, _ = arm.start('return_home', {})
    assert status == 202 and arm.resume is None                                              # 새 작업을 시키면 버린다


def test_chat_resume_proposes_resume_only_when_something_is_stopped():
    arm = arm_api.Arm()
    quiet = Mock(side_effect=AssertionError('No LLM needed'))
    phrases = ('다시 시작해', '이어서 해줘', '계속해', '재개', '다시 작업해줘', '작업 다시 해줘', '이어서 작업해')
    for text in phrases:
        none = arm_llm.chat([], text, arm, client=quiet)
        assert none['proposal'] is None and '이어서 할 작업이 없습니다' in none['reply']      # 정지한 작업도, 마지막 작업도 없다
    arm.resume = dict(skill='pick_all', params=dict(objects=['white_box']))
    for text in phrases:
        result = arm_llm.chat([], text, arm, client=quiet)
        assert result['proposal'] == {'action': 'resume'} and '책상 정리(흰 상자)' in result['reply']
    assert arm.resume_summary() == '책상 정리(흰 상자)'


def test_resume_without_a_stopped_task_repeats_the_last_task():
    # 2026-10-28: '다시 시작해'에 '이어서 할 작업이 없습니다', '다시 작업해줘'에 AI가 엉뚱하게 되물었다
    arm = arm_api.Arm()
    arm.tasks['x'] = dict(id='x', skill='pick_place', params=dict(object='white_box', slot=2), state='failed', started=1., seconds=3.,
                          result=None, phase=None, phase_label=None, item=None, items=[], phases=[], warnings=[])
    quiet = Mock(side_effect=AssertionError('No LLM needed'))
    for text in ('다시 시작해', '다시 작업해줘'):
        result = arm_llm.chat([], text, arm, client=quiet)
        assert result['proposal'] == {'action': 'pick_place', 'object': 'white_box'} and '흰 상자' in result['reply']


def test_status_tells_whether_resumable():
    arm = arm_api.Arm()
    assert arm.status()['resumable'] is False
    arm.resume = dict(skill='drive', params=dict(label='책상'))
    assert arm.status()['resumable'] is True and arm.resume_summary() == '책상으로 이동'


def test_resume_skips_the_home_trip_when_the_arm_is_already_home_after_the_stop():
    run = Mock()
    arm = _arm(run)
    arm.arm_home = True                                                                    # 정지 뒤 자동 복귀가 끝났다
    arm.resume = dict(skill='pick_place', params=dict(object='white_box', slot=2))
    arm._execute = Mock(return_value=dict(ok=True, reason=None, reason_code=None))
    arm._resume('t')
    run.assert_not_called()
    arm._execute.assert_called_once_with('t', 'pick_place', dict(object='white_box', slot=2))
