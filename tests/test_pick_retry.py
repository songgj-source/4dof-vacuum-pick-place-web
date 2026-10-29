"""집기가 실패하면 HOME으로 돌아가 다시 시도한다(2026-10-28 사용자 요청). 하드웨어 접근 없음."""
import os
from unittest.mock import patch

import arm_api


def _arm(results, home_ok=True):
    events, calls = [], []
    arm = arm_api.Arm(read_state=lambda: 'ok', base_check=lambda: None)
    arm._event = lambda task, kind, data: events.append(data)
    arm._ensure_torque = lambda *a: None
    queue = iter(results)
    arm._pick_once = lambda task, label, slot, which=None, of=None: calls.append(('pick', label, slot)) or next(queue)
    arm._run = lambda task, cmd: calls.append(('home',)) or ((0, {'home_return_complete': True, 'ok': True}, 1.) if home_ok else (1, {}, 1.))
    return arm, events, calls


FAIL = {'ok': False, 'reason': 'not held', 'reason_code': 'not_held'}
OK = {'ok': True, 'reason': None, 'reason_code': None}


def test_failed_pick_goes_home_and_tries_again():
    arm, events, calls = _arm([FAIL, OK])
    with patch.dict(os.environ, {'ARM_PICK_RETRY': '2'}), patch.object(arm_api.menu, 'home_return_succeeded', return_value=True), patch.object(arm_api.CAMERA, 'release'):
        result = arm._pick('task', 'red_cube', 1)
    assert result['ok'] and [c[0] for c in calls] == ['pick', 'home', 'pick']
    assert ('pick_retry', '빨간 정육면체 집기에 실패했습니다. HOME으로 돌아가 다시 시도합니다(1/2).') in events


def test_retries_are_bounded_and_skip_hopeless_failures():
    arm, events, calls = _arm([FAIL] * 5)
    with patch.dict(os.environ, {'ARM_PICK_RETRY': '2'}), patch.object(arm_api.menu, 'home_return_succeeded', return_value=True), patch.object(arm_api.CAMERA, 'release'):
        result = arm._pick('task', 'red_cube', 1)
    assert not result['ok'] and sum(c[0] == 'pick' for c in calls) == 3          # 처음 + 2번
    for reason in ({'ok': False, 'reason': 'not reachable', 'reason_code': 'unreachable'}, {'ok': False, 'reason': 'cancelled', 'reason_code': 'cancelled'},
                   {'ok': False, 'reason': 'x', 'reason_code': 'torque_off'}):
        arm, events, calls = _arm([reason])
        with patch.dict(os.environ, {'ARM_PICK_RETRY': '2'}), patch.object(arm_api.CAMERA, 'release'):
            assert not arm._pick('task', 'red_cube', 1)['ok']
        assert calls == [('pick', 'red_cube', 1)]


def test_no_retry_when_home_return_fails_or_stop_requested_or_disabled():
    arm, events, calls = _arm([FAIL, OK], home_ok=False)
    with patch.dict(os.environ, {'ARM_PICK_RETRY': '2'}), patch.object(arm_api.menu, 'home_return_succeeded', return_value=False), patch.object(arm_api.CAMERA, 'release'):
        assert not arm._pick('task', 'red_cube', 1)['ok']
    assert [c[0] for c in calls] == ['pick', 'home']
    arm, events, calls = _arm([FAIL, OK])
    arm.stop_requested = True
    with patch.dict(os.environ, {'ARM_PICK_RETRY': '2'}):
        arm._pick('task', 'red_cube', 1)
    assert calls == [('pick', 'red_cube', 1)]
    arm, events, calls = _arm([FAIL, OK])
    assert not arm._pick('task', 'red_cube', 1)['ok'] and len(calls) == 1          # 시험 기본값 ARM_PICK_RETRY=0


def test_registered_pick_all_retries_from_the_failed_object_with_the_rest():
    arm = arm_api.Arm(read_state=lambda: 'ok', base_check=lambda: None)
    events, runs = [], []
    arm._event = lambda task, kind, data: events.append(data)
    arm._ensure_torque = lambda *a: None
    arm.tasks['task'] = {}
    outcomes = iter([
        ({'ok': False, 'reason': '흰 상자에서 멈췄습니다', 'reason_code': 'not_held', 'failed_label': 'white_box', '_placed': [1]}, [dict(object='red_cube', ok=True)]),
        ({'ok': True, 'reason': None, 'reason_code': None, 'note': None, '_placed': [2, 3]}, [dict(object='white_box', ok=True), dict(object='tape_measure', ok=True)]),
    ])
    def once(task, objects=None, include_new=False):
        runs.append(objects)
        result, items = next(outcomes)
        arm.tasks['task']['items'] = items
        return result
    arm._pick_registered_once = once
    arm._home_for_retry = lambda task: True
    with patch.dict(os.environ, {'ARM_PICK_RETRY': '2'}):
        result = arm._pick_registered('task', ['red_cube', 'white_box', 'tape_measure'], False)
    assert runs == [['red_cube', 'white_box', 'tape_measure'], ['white_box', 'tape_measure']]       # 끝낸 정육면체는 빼고 실패한 물체부터
    assert result['ok'] and result['_placed'] == [1, 2, 3]
    assert [i['object'] for i in arm.tasks['task']['items']] == ['red_cube', 'white_box', 'tape_measure']
    assert any(e[0] == 'pick_retry' and '흰 상자' in e[1] for e in events)
