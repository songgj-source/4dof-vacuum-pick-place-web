"""도착한 뒤 한 번 더 찍고 상황을 설명한다(2026-10-28 사용자 요청). 하드웨어 접근 없음."""
import os
from unittest.mock import patch

import arm_api
import arm_llm

SCENE = {'objects': [{'label': 'tape_measure', 'pickable': False, 'range': 'near', 'distance_mm': 120}, {'label': 'red_cube', 'pickable': True},
                     {'label': 'unknown', 'reachable': True, 'color': '갈색', 'guess': '상자'},
                     {'label': 'unknown', 'reachable': False}],
         'in_tray': [{'label': 'red_cube', 'name': '빨간 정육면체', 'slot': 1}, {'label': 'white_box', 'name': '흰 상자', 'slot': 2}]}


def test_report_says_desk_tray_unreachable_and_plan():
    text = arm_llm.arrival_report('책상', SCENE)
    assert text.startswith('책상에 도착해 다시 찍었습니다.')
    assert '책상 위: 빨간 정육면체, 줄자, 새 물체 2개(갈색 상자 같음).' in text
    assert '칸 안: 빨간 정육면체(1번 칸), 흰 상자(2번 칸).' in text
    assert '줄자는 너무 가깝습니다(12cm, 최소 16cm)' in text
    assert text.endswith('빨간 정육면체, 새 물체 1개를 정리하겠습니다.')
    assert '정리하겠습니다' not in arm_llm.arrival_report('책상', SCENE, tidy=False)
    assert '책상 위에 물체가 없습니다.' in arm_llm.arrival_report('책상', {'objects': [], 'in_tray': []})


def _arm(scene):
    events = []
    arm = arm_api.Arm(read_state=lambda: 'ok', arrival_check=lambda: None,
                      read_pose=lambda: {n: (1, v) for n, v in arm_api.menu.HOME_RAW.items()},
                      drive=lambda *a, **k: {'ok': True})
    arm._event = lambda task, kind, data: events.append(data)
    arm._ensure_torque = lambda *a: None
    arm._pick_all = lambda *a, **k: events.append(('pick', None)) or {'ok': True}
    arm._steady_scene = lambda grab: (None, scene)
    return arm, events


def test_tidy_arrival_announces_report_before_picking():
    arm, events = _arm(SCENE)
    with patch.object(arm_api.time, 'sleep'), patch.object(arm_api.CAMERA, 'release'):
        result = arm._drive('task', {'label': '책상', 'then': 'tidy'})
    found = [e for e in events if e[0] == 'tidy_found']
    assert found and '칸 안: 빨간 정육면체(1번 칸)' in found[0][1]
    assert events.index(found[0]) < events.index(('pick', None))
    assert result['announcement'] == found[0][1]


def test_plain_drive_photographs_and_describes_without_arm_motion():
    arm, events = _arm(SCENE)
    with patch.dict(os.environ, {'ARM_ARRIVAL_PHOTO': '1'}), patch.object(arm_api.time, 'sleep'), \
            patch.object(arm_api.CAMERA, 'release'):
        result = arm._drive('task', {'label': '책상'})
    assert result['ok'] and '책상 위: 빨간 정육면체' in result['note']
    assert any(e[0] == 'arrival_report' for e in events)
    assert ('pick', None) not in events


def test_plain_drive_stays_successful_when_photo_fails():
    arm, events = _arm(SCENE)
    def broken(grab):
        raise OSError('Astra 없음')
    arm._steady_scene = broken
    with patch.dict(os.environ, {'ARM_ARRIVAL_PHOTO': '1'}), patch.object(arm_api.time, 'sleep'), \
            patch.object(arm_api.CAMERA, 'release'):
        result = arm._drive('task', {'label': '책상'})
    assert result['ok'] and '상황을 확인하지 못했습니다' in result['note']
