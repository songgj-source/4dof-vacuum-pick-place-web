"""Nav2가 시작 때 비활성으로 남아 모든 주행 목표를 거부하던 문제(2026-10-28): 서버가 내비게이션만 다시 켜고 한 번 더 시도한다."""
import sys
from pathlib import Path
from unittest.mock import Mock

sys.path[:0] = [str(Path(__file__).resolve().parents[1] / 'scripts'), str(Path(__file__).resolve().parents[1] / 'src')]
import arm_api
import base_drive

REJECTED = '시작위치으로 가지 못했습니다: Nav2가 목표를 거부했습니다'


def _arm(drive, recover):
    arm = arm_api.Arm(read_state=lambda: 'ok', base_check=lambda: None, drive=drive,
                      read_pose=lambda: {n: (1, v) for n, v in arm_api.menu.HOME_RAW.items()})
    arm.tasks['t'] = dict(id='t', skill='drive', params={}, state='running', phase=None, phase_label=None, phases=[], warnings=[],
                          result=None, item=None, items=[], started=0., seconds=0.)
    arm.current = 't'
    arm._recover_nav = recover
    return arm


def _go(arm):
    return arm._drive('t', dict(label='시작위치', mode='precise'))


def test_nav2_rejection_triggers_recovery_then_one_retry_that_succeeds():
    drive = Mock(side_effect=[base_drive.BaseError(REJECTED, 'drive_failed'), dict(ok=True)])
    recover = Mock(return_value=True)
    result = _go(_arm(drive, recover))
    assert result['ok'] and drive.call_count == 2 and recover.call_count == 1


def test_failed_recovery_says_so_and_does_not_retry():
    drive = Mock(side_effect=base_drive.BaseError(REJECTED, 'drive_failed'))
    result = _go(_arm(drive, Mock(return_value=False)))
    assert not result['ok'] and drive.call_count == 1 and '다시 켜지 못했습니다' in result['reason'] and result['reason_code'] == 'drive_failed'


def test_other_drive_errors_are_not_retried():
    drive = Mock(side_effect=base_drive.BaseError('장애물 때문에 길을 찾지 못했습니다', 'drive_failed'))
    recover = Mock(return_value=True)
    result = _go(_arm(drive, recover))
    assert not result['ok'] and drive.call_count == 1 and recover.call_count == 0


def test_no_recovery_when_the_user_already_stopped():
    drive = Mock(side_effect=base_drive.BaseError(REJECTED, 'drive_failed'))
    recover = Mock(return_value=True)
    arm = _arm(drive, recover)
    arm.stop_requested = True
    assert not _go(arm)['ok'] and recover.call_count == 0


def test_drive_ctl_checks_nav_readiness_after_localizing_and_has_a_manual_command():
    script = (Path(__file__).resolve().parents[1] / 'pi' / 'drive_ctl.sh').read_text()
    start = script[script.index('start() {'):script.index('# 지도 위 위치를 한 번 잡는다')]
    assert start.index('  relocalize\n') < start.index('  nav_ready\n')                      # 위치를 잡은 뒤에 Nav2 활성을 확인
    assert 'nav-ready) nav_ready ;;' in script and 'command: 3' in script and 'command: 0' in script   # 초기화 후 다시 켜기
