"""책상 정리에서 새 물체(등록 안 된 물체)를 기존 물체가 쓰고 남은 빈 칸에 넣는다. 책상에는 물체가 항상 3개다."""
import sys
from pathlib import Path
from unittest.mock import Mock, patch

sys.path[:0] = [str(Path(__file__).resolve().parents[1] / 'scripts'), str(Path(__file__).resolve().parents[1] / 'src')]
import arm_api

OK = {'ok': True, 'reason': None, 'reason_code': None}


def _arm(new_objects, placed, in_tray=(), pick=None):
    arm = arm_api.Arm()
    arm.tasks['t'] = {}
    arm._event = Mock()
    arm._ensure_torque = lambda *a: None
    scene = dict(objects=[dict(label='unknown', pixel=[x, 240], height_mm=h, guess=g) for x, h, g in new_objects],
                 in_tray=[dict(slot=s, label='red_cube') for s in in_tray], tray_found=True)
    arm._steady_scene = lambda grab: (None, scene)
    arm._pick_registered = Mock(return_value=dict(OK, note=None, _placed=list(placed)))
    arm._pick = pick or Mock(return_value=dict(OK))
    return arm


def _run(arm, objects=('red_cube',)):
    with patch.object(arm_api.CAMERA, 'release'):
        return arm._pick_all('t', list(objects), include_new=True)


def test_two_registered_plus_one_new_fills_the_only_free_slot():
    arm = _arm([(300, 30., '박스')], placed=[1, 3])
    result = _run(arm, ('red_cube', 'tape_measure'))
    arm._pick.assert_called_once_with('t', 'unknown', 2, which=1, of=1)
    assert result['ok'] and '새 물체 1개를 넣었습니다' in result['note']


def test_one_registered_plus_two_new_taller_goes_to_far_slot_and_right_one_is_picked_first():
    arm = _arm([(200, 24., '상자'), (300, 45., '병')], placed=[2])         # 왼쪽 낮은 것 = 1번, 오른쪽 높은 것 = 2번
    result = _run(arm, ('white_box',))
    # 높은 물체가 더 멀리 닿는다고 계산되어 가장 먼 빈 칸(1번)으로, 오른쪽(2번)부터 집는다
    assert arm._pick.call_args_list[0].args == ('t', 'unknown', 1) and arm._pick.call_args_list[0].kwargs == dict(which=2, of=2)
    assert arm._pick.call_args_list[1].args == ('t', 'unknown', 3) and arm._pick.call_args_list[1].kwargs == dict(which=1, of=1)
    assert result['ok'] and '새 물체 2개를 넣었습니다' in result['note']


def test_three_new_objects_use_all_three_slots_once():
    arm = _arm([(200, 30., None), (300, 35., None), (400, 40., None)], placed=[])
    _run(arm, ())                                              # 기존 물체 없음(objects=[])
    arm._pick_registered.assert_not_called()
    slots = sorted(call.args[2] for call in arm._pick.call_args_list)
    assert slots == [1, 2, 3]
    assert [call.kwargs['which'] for call in arm._pick.call_args_list] == [3, 2, 1]        # 오른쪽부터


def test_same_scene_gives_same_assignment():
    scene = [(200, 30., None), (300, 35., None), (400, 40., None)]
    first = _arm(scene, placed=[]); _run(first, ())
    second = _arm(scene, placed=[]); _run(second, ())
    assert first._pick.call_args_list == second._pick.call_args_list


def test_a_failed_new_object_stops_the_rest_and_says_which():
    arm = _arm([(200, 24., None), (300, 45., None)], placed=[2], pick=Mock(return_value=dict(ok=False, reason='suction failed', reason_code='x')))
    result = _run(arm, ('white_box',))
    assert not result['ok'] and '새 물체 2에서 멈췄습니다' in result['reason']
    assert arm._pick.call_count == 1


def test_no_new_objects_changes_nothing():
    arm = _arm([], placed=[1, 2, 3])
    result = _run(arm, ('red_cube', 'white_box', 'tape_measure'))
    arm._pick.assert_not_called()
    assert result['ok']


def test_registered_failure_skips_new_objects():
    arm = _arm([(300, 30., None)], placed=[])
    arm._pick_registered = Mock(return_value=dict(ok=False, reason='stuck', reason_code='x', _placed=[]))
    result = _run(arm)
    arm._pick.assert_not_called()
    assert not result['ok']


def test_start_accepts_include_new_with_empty_objects_only_when_flagged():
    arm = arm_api.Arm()
    for params, expect_ok in (({'objects': [], 'include_new': True}, True), ({'objects': []}, False), ({'include_new': 'yes'}, False)):
        with patch.object(arm_api.threading, 'Thread') as thread:          # 실제 작업(_work)을 시작하지 않는다
            status, body = arm.start('pick_all', params)
        assert (status < 400) == expect_ok, (params, status, body)
        assert thread.return_value.start.called == expect_ok
