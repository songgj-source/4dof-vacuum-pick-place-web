"""A desk request drives, stops, photographs, announces objects, then sorts."""
import sys
from pathlib import Path
from unittest.mock import Mock, patch
import numpy as np

sys.path[:0] = [str(Path(__file__).resolve().parents[1] / 'scripts'), str(Path(__file__).resolve().parents[1] / 'src')]
import arm_api
import arm_llm


def _desk_chat(capture, text='책상 정리해줘'):
    arm = arm_api.Arm()
    arm.capture = Mock(return_value=capture)
    with patch.object(arm_llm.base_drive, 'base_state', return_value={'labels': [{'id': 'desk', 'name': '책상'}]}):
        return arm_llm.chat([], text, arm, client=Mock(side_effect=AssertionError('No LLM needed'))), arm


def test_desk_request_photographs_first_and_tidies_in_place_when_already_at_desk():
    scene = {'objects': [{'label': 'red_cube', 'pickable': True}], 'in_tray': [], 'tray_found': True}
    result, arm = _desk_chat((200, scene))
    arm.capture.assert_called_once()
    assert result['proposal'] == {'action': 'pick_all', 'objects': ['red_cube']}      # 이동 없이 바로 정리
    assert result['scene'] is scene


def test_desk_request_drives_to_desk_when_photo_shows_no_desk():
    scene = {'objects': [], 'in_tray': [], 'tray_found': False}
    result, arm = _desk_chat((200, scene))
    arm.capture.assert_called_once()
    assert result['proposal'] == {'action': 'drive', 'label': '책상', 'then': 'tidy'}
    assert '책상이 보이지 않습니다' in result['reply']


def test_only_unregistered_objects_do_not_count_as_desk():
    scene = {'objects': [{'label': 'unknown', 'pickable': False}], 'in_tray': [], 'tray_found': False}
    result, _ = _desk_chat((200, scene))
    assert result['proposal'] == {'action': 'drive', 'label': '책상', 'then': 'tidy'}


def test_desk_in_view_but_nothing_to_tidy_does_not_move():
    scene = {'objects': [], 'in_tray': [], 'tray_found': True}
    result, _ = _desk_chat((200, scene))
    assert result['proposal'] is None and '정리할 물체가 없습니다' in result['reply']


def test_photo_failure_falls_back_to_driving_to_desk():
    result, _ = _desk_chat((503, {'reason': 'camera_busy'}))
    assert result['proposal'] == {'action': 'drive', 'label': '책상', 'then': 'tidy'}
    assert '사진을 찍지 못해' in result['reply'] and result['scene'] is None


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
    arm._pick_all.assert_called_once_with('task', ['red_cube'], include_new=False)


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
    arm._pick_all.assert_called_once_with(body['task_id'], ['red_cube'], include_new=False)


def test_negative_or_question_desk_text_never_starts_driving():
    arm = arm_api.Arm()
    with patch.object(arm_llm.base_drive, 'base_state', side_effect=AssertionError('Must not request travel')):
        for text in ['책상 정리하지마', '책상 정리해줄까?']:
            result = arm_llm.chat([], text, arm, client=Mock(side_effect=AssertionError('No LLM needed')))
            assert result['proposal'] is None
