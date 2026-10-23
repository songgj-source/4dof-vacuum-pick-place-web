"""The combined task may photograph and pick only after confirmed continuous stop."""
import sys
from pathlib import Path
from unittest.mock import Mock, patch

sys.path[:0] = [str(Path(__file__).resolve().parents[1] / 'scripts'),
                str(Path(__file__).resolve().parents[1] / 'src')]
import arm_api
import base_drive


def healthy_drive():
    return {'driver_connected': True, 'moving': False, 'odom': {'linear': 0., 'angular': 0.}}


def test_stop_check_refuses_missing_speed_or_disconnected_driver():
    for drive in ({**healthy_drive(), 'odom': None},
                  {**healthy_drive(), 'driver_connected': False},
                  {**healthy_drive(), 'odom': {'linear': float('nan'), 'angular': 0.}}):
        with patch.object(base_drive, '_call', side_effect=[drive, {'nav': {'state': 'succeeded'}}]):
            assert arm_api.base_stationary() is not None


def test_stop_check_refuses_unreachable_server():
    with patch.object(base_drive, '_call', side_effect=base_drive.BaseError('offline')):
        assert arm_api.base_stationary() is not None


def task_arm(arrivals, stop_after_check=False):
    events = []
    arm = arm_api.Arm(read_state=lambda: 'ok')
    checks = iter(arrivals)
    def check():
        result = next(checks)
        events.append(('stop_check', result))
        if stop_after_check:
            arm.stop_requested = True
        return result
    arm.arrival_check = check
    arm._event = lambda *args: None
    arm._ensure_torque = lambda *args: None
    arm._pick_all = lambda *args: events.append(('pick', None)) or {'ok': True}
    def capture(grab):
        events.append(('photo', None))
        return None, {'objects': [{'label': 'red_cube', 'pickable': True}]}
    arm._steady_scene = capture
    return arm, events


def test_stop_samples_reset_when_movement_reappears():
    arm, events = task_arm([None, 'moving', None, None, None])
    with patch.object(arm_api.time, 'sleep'), patch.object(arm_api.CAMERA, 'release'):
        result = arm._tidy_here('task', '책상')
    assert result['ok']
    assert [e[0] for e in events] == ['stop_check'] * 5 + ['photo', 'pick']


def test_uncertain_stop_never_photographs_or_picks():
    arm, events = task_arm(['offline'] * 8)
    with patch.object(arm_api.time, 'sleep'):
        result = arm._tidy_here('task', '책상')
    assert not result['ok']
    assert not any(e[0] in ('photo', 'pick') for e in events)


def test_cancellation_during_stop_wait_never_photographs():
    arm, events = task_arm([None] * 8, stop_after_check=True)
    with patch.object(arm_api.time, 'sleep'):
        result = arm._tidy_here('task', '책상')
    assert result['reason_code'] == 'cancelled'
    assert not any(e[0] in ('photo', 'pick') for e in events)


def test_failed_drive_never_enters_camera_stage():
    arm = arm_api.Arm(read_pose=lambda: {n: (1, v) for n, v in arm_api.menu.HOME_RAW.items()},
                      drive=lambda *a, **k: {'ok': False, 'reason_code': 'drive_failed'})
    arm._tidy_here = Mock()
    result = arm._drive('task', {'label': '책상', 'then': 'tidy'})
    assert not result['ok']
    arm._tidy_here.assert_not_called()
