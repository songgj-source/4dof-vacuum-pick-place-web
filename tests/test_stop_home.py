"""정지 명령을 보내면 정지가 끝난 뒤 HOME으로 복귀한다(집기·정리만, 복귀 중 또 정지하면 복귀도 멈춘다). ARM_HOME_AFTER_STOP=0이면 그 자리에 선다."""
import sys
from pathlib import Path
from unittest.mock import Mock

sys.path[:0] = [str(Path(__file__).resolve().parents[1] / 'scripts'), str(Path(__file__).resolve().parents[1] / 'src')]
import arm_api
import arm_llm

CANCELLED = dict(ok=False, reason='cancelled', reason_code='cancelled')
HOME_OK = dict(home_return_complete=True, final_hold_confirmed=True, restoration_verified=True)


def _arm(run, **kw):
    arm = arm_api.Arm(run=run, read_state=lambda: 'ok', base_check=lambda: None, **kw)
    arm.tasks['t'] = dict(id='t', skill='pick_place', params={}, state='running', phase=None, phase_label=None, phases=[], warnings=[],
                          result=None, item=None, items=[], started=0., seconds=0.)
    arm.current = 't'
    return arm


def _stopped_work(arm, skill='pick_place', params=None):
    arm._execute = Mock(return_value=dict(CANCELLED))
    arm.stop_requested = True                                         # 정지가 막 작업을 멈춘 상태
    arm._work('t', skill, params or {})
    return arm.tasks['t']['result']


def _run_returning(code, summary, calls):
    def run(command, **kw):
        calls.append(command)
        return code, summary, 1.
    return run


def test_stopped_pick_returns_home_afterwards():
    calls = []
    arm = _arm(_run_returning(0, dict(HOME_OK), calls))
    result = _stopped_work(arm)
    assert [Path(c[1]).name for c in calls] == ['return_home.py'] and '--any-start' in calls[0]
    assert result['home_after_stop']['ok'] and not result['ok'] and result['reason_code'] == 'cancelled'      # 작업은 '정지됨', 복귀는 따로 성공
    assert '정지 후 HOME 복귀를 마쳤습니다' in result['note'] and arm.arm_home is True


def test_failed_home_is_reported_not_hidden():
    arm = _arm(_run_returning(1, dict(error='sampled link endpoint/table clearance failed'), []))
    result = _stopped_work(arm)
    assert not result['home_after_stop']['ok'] and 'clearance failed' in result['note'] and arm.arm_home is None


def test_second_stop_during_return_stops_the_return_too():
    calls = []
    def run(command, **kw):
        calls.append(command)
        return 130, dict(ok=False, reason='cancelled'), 1.
    arm = _arm(run)
    arm._ensure_torque = lambda *a: setattr(arm, 'stop_requested', True)          # 복귀 직전에 또 정지가 들어왔다
    result = _stopped_work(arm)
    assert calls == [] and result['home_after_stop']['reason'] == 'cancelled'


def test_no_home_for_drive_return_home_or_when_disabled():
    for skill in ('drive', 'return_home'):
        calls = []
        result = _stopped_work(_arm(_run_returning(0, dict(HOME_OK), calls)), skill=skill)
        assert calls == [] and 'home_after_stop' not in result
    calls = []
    result = _stopped_work(_arm(_run_returning(0, dict(HOME_OK), calls), home_after_stop=False))
    assert calls == [] and 'home_after_stop' not in result


def test_normal_finish_does_not_go_home_by_itself():
    calls = []
    arm = _arm(_run_returning(0, dict(HOME_OK), calls))
    arm._execute = Mock(return_value=dict(ok=True, reason=None, reason_code=None))
    arm._work('t', 'pick_place', {})
    assert calls == []


def test_default_goes_home_and_env_zero_stays_in_place(monkeypatch):
    monkeypatch.delenv('ARM_HOME_AFTER_STOP', raising=False)
    assert arm_api.Arm().home_after_stop is True                                # 기본: 정지 뒤 HOME 복귀
    monkeypatch.setenv('ARM_HOME_AFTER_STOP', '0')
    arm = arm_api.Arm(run=_run_returning(0, dict(HOME_OK), calls := []), read_state=lambda: 'ok', base_check=lambda: None)
    assert arm.home_after_stop is False
    arm.tasks['t'] = dict(id='t', skill='pick_place', params={}, state='running', phase=None, phase_label=None, phases=[], warnings=[],
                          result=None, item=None, items=[], started=0., seconds=0.)
    arm.current = 't'
    result = _stopped_work(arm)
    assert calls == [] and 'home_after_stop' not in result                      # 0이면 그 자리에 선다


def test_stop_reply_says_home_and_how_to_resume():
    assert arm_llm.stop_reply(dict(stopped=True, resumable=True, home_after_stop=True)) == (
        "멈춥니다. 멈춘 뒤 HOME으로 돌아갑니다. '다시 시작해'라고 하면 남은 일을 이어서 합니다.")
    assert arm_llm.stop_reply(dict(stopped=True, resumable=False, home_after_stop=True)) == '멈춥니다. 멈춘 뒤 HOME으로 돌아갑니다.'
    assert '그 자리에 섭니다' in arm_llm.stop_reply(dict(stopped=True, resumable=False, home_after_stop=False))
    assert arm_llm.stop_reply(dict(stopped=False)) == '지금 하고 있는 작업이 없습니다.'
    arm = _arm(_run_returning(0, {}, []))
    outcome = arm.stop()
    assert outcome['stopped'] and outcome['resumable'] is True and outcome['home_after_stop'] is True
    drive = _arm(_run_returning(0, {}, [])); drive.tasks['t']['skill'] = 'drive'
    assert drive.stop()['home_after_stop'] is False and drive.stop()['resumable'] is True       # 주행은 팔을 HOME에 보내지 않는다


def test_stop_that_kills_the_run_by_signal_is_shown_as_stopped_not_as_a_failure():
    # 2026-10-28: 정지하면 집기 프로세스가 종료 코드 -2로 끝나 '흰 상자에서 멈췄습니다: 종료 코드 -2'가 화면에 나왔다
    arm = _arm(_run_returning(0, dict(HOME_OK), []))
    arm._execute = Mock(return_value=dict(ok=False, reason='흰 상자에서 멈췄습니다: 종료 코드 -2', reason_code='failed'))
    arm.stop_requested = True
    arm._work('t', 'pick_all', dict(objects=['white_box']))
    result = arm.tasks['t']['result']
    assert (result['reason'], result['reason_code']) == ('cancelled', 'cancelled') and '종료 코드' not in result['reason']
    assert result['home_after_stop']['ok']                                       # 정지로 봤으니 HOME 복귀도 한다
    assert arm.resume == dict(skill='pick_all', params=dict(objects=['white_box']))      # 이어서 하기도 기억된다


def test_a_failure_without_a_stop_is_still_a_failure():
    arm = _arm(_run_returning(0, dict(HOME_OK), []))
    arm._execute = Mock(return_value=dict(ok=False, reason='흰 상자에서 멈췄습니다: 흡착 실패', reason_code='failed'))
    arm._work('t', 'pick_all', dict(objects=['white_box']))
    result = arm.tasks['t']['result']
    assert result['reason_code'] == 'failed' and '흡착 실패' in result['reason'] and arm.resume is None


def test_web_texts_for_stop(tmp_path):
    html = (Path(__file__).resolve().parents[1] / 'web/index.html').read_text()
    assert "if (phase === 'home_after_stop') return ['stopping', '작업을 중지합니다.']" in html
    assert html.index("phase === 'home_after_stop'") < html.index("phase.includes('home')")         # 일반 'HOME 복귀' 문구보다 먼저
    assert "'작업을 중지했습니다.'" in html and "'정지 후 HOME 복귀에 실패했습니다: '" in html
    assert "'멈췄습니다.' :" not in html                                                              # 정지 결과를 '멈췄습니다'로 내지 않는다
