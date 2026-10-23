"""A desk request drives, stops, photographs, announces objects, then sorts."""
import sys
from pathlib import Path
from unittest.mock import Mock, patch
import numpy as np

sys.path[:0] = [str(Path(__file__).resolve().parents[1] / 'scripts'), str(Path(__file__).resolve().parents[1] / 'src')]
import arm_api
import arm_llm


def test_short_desk_request_drives_to_sort_without_predeparture_photo():
    arm = arm_api.Arm()
    arm.capture = Mock(side_effect=AssertionError('No photo before arrival'))
    with patch.object(arm_llm.base_drive, 'base_state', return_value={'labels': [{'id': 'desk', 'name': '책상'}]}):
        result = arm_llm.chat([], '책상 정리해줘', arm, client=Mock(side_effect=AssertionError('No LLM needed')))
    assert result['proposal'] == {'action': 'drive', 'label': '책상', 'then': 'tidy'}
    arm.capture.assert_not_called()


def test_arrival_announces_visible_objects_before_sorting():
    events = []
    arm = arm_api.Arm(arrival_check=lambda: None)
    arm._event = lambda task,kind,data: events.append(data[0])
    arm.read_state = lambda: 'ok'
    arm._pick_all = Mock(return_value={'ok': True, 'reason': None, 'reason_code': None})
    scene = {'objects': [{'label': 'red_cube', 'pickable': True}], 'in_tray': []}
    arm._steady_scene = lambda grab: (np.zeros((3,3,3), dtype=np.uint8), scene)
    with patch.object(arm_api.time, 'sleep'), patch.object(arm_api.CAMERA, 'release'):
        result = arm._tidy_here('task', '책상', ask=True)
    assert result['ok'] and not result.get('awaiting_choice')
    assert events.index('drive_settle') < events.index('tidy_look') < events.index('tidy_found')
    assert '책상에 빨간 정육면체가 보입니다. 정리하겠습니다.' in result['announcement']
    arm._pick_all.assert_called_once_with('task', ['red_cube'])


def test_empty_desk_does_not_sort_or_ask():
    arm = arm_api.Arm(arrival_check=lambda: None)
    arm._event = lambda task, kind, data: None
    arm._pick_all = Mock(side_effect=AssertionError('Empty desk must not sort'))
    arm._steady_scene = lambda grab: (np.zeros((3, 3, 3), dtype=np.uint8), {'objects': [], 'in_tray': []})
    with patch.object(arm_api.time, 'sleep'), patch.object(arm_api.CAMERA, 'release'):
        result = arm._tidy_here('task', '책상', ask=True)
    assert result['ok'] and '정리할 물체가 없습니다' in result['note']
    assert not result.get('awaiting_choice')


def test_specific_answer_is_an_arm_command_not_another_drive():
    arm = arm_api.Arm()
    arm.awaiting_desk_choice = True
    result = arm_llm.chat([], '빨간 정육면체 넣어줘', arm, client=Mock(side_effect=AssertionError('No LLM needed')))
    assert result['proposal'] == {'action': 'pick_place', 'object': 'red_cube'}


def test_desk_task_finishes_sorting_without_asking_user():
    arm = arm_api.Arm(run=Mock(),
                      read_state=lambda: 'ok', base_check=lambda: None,
                      arrival_check=lambda: None,
                      read_pose=lambda: {n:(1,v) for n,v in arm_api.menu.HOME_RAW.items()},
                      drive=lambda *a,**k: {'ok': True})
    scene = {'objects': [{'label': 'red_cube', 'pickable': True}], 'in_tray': []}
    arm._steady_scene = lambda grab: (np.zeros((3,3,3), dtype=np.uint8), scene)
    arm._pick_all = Mock(return_value={'ok': True, 'reason': None, 'reason_code': None})
    with patch.object(arm_api.threading.Thread, 'start'), patch.object(arm_api.menu, 'live_view_running', return_value=False):
        status, body = arm.start('drive', {'label': '책상', 'then': 'tidy'})
    assert status == 202
    with patch.object(arm_api.time, 'sleep'), patch.object(arm_api.CAMERA, 'release'):
        arm._work(body['task_id'], 'drive', {'label': '책상', 'mode':'precise', 'then': 'tidy'})
    result = arm.task(body['task_id'])
    assert result['state'] == 'done' and not result['result'].get('awaiting_choice')
    assert arm.status()['state'] == 'idle'
    assert arm.arm_home is True
    arm._pick_all.assert_called_once_with(body['task_id'], ['red_cube'])


def test_negative_or_question_desk_text_never_starts_driving():
    arm = arm_api.Arm()
    with patch.object(arm_llm.base_drive, 'base_state', side_effect=AssertionError('Must not request travel')):
        for text in ['책상 정리하지마', '책상 정리해줄까?']:
            result = arm_llm.chat([], text, arm, client=Mock(side_effect=AssertionError('No LLM needed')))
            assert result['proposal'] is None
