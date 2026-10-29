"""정리를 마친 뒤 책상을 다시 찍어 확인한다(2026-10-28 사용자 요청). 하드웨어 접근 없음."""
import os
from unittest.mock import patch

import arm_api
import arm_llm

GOOD = {'objects': [], 'in_tray': [{'label': 'red_cube', 'name': '빨간 정육면체', 'slot': 1}, {'label': 'white_box', 'name': '흰 상자', 'slot': 2},
                    {'label': 'unknown', 'name': '물체', 'slot': 3}], 'tray_found': True}


def test_report_good_left_and_wrong_slot():
    ok, text = arm_llm.tidy_check_report(GOOD)
    assert ok and text.startswith('정리 확인: 책상에 남은 물체 없이 잘 정리됐습니다.') and '새 물체(3번 칸)' in text
    left = dict(GOOD, objects=[{'label': 'tape_measure', 'name': '줄자', 'pickable': False, 'range': 'near', 'distance_mm': 120}])
    ok, text = arm_llm.tidy_check_report(left)
    assert not ok and '정리가 덜 됐습니다' in text and '책상에 남은 물체: 줄자.' in text and '너무 가깝습니다(12cm, 최소 16cm)' in text
    swapped = dict(GOOD, in_tray=[{'label': 'red_cube', 'name': '빨간 정육면체', 'slot': 2}])
    ok, text = arm_llm.tidy_check_report(swapped)
    assert not ok and '빨간 정육면체가 2번 칸에 있습니다(기본 1번 칸)' in text
    ok, text = arm_llm.tidy_check_report({'objects': [], 'in_tray': [], 'tray_found': False})
    assert not ok and '확인하지 못했습니다' in text


def _arm(scene):
    events = []
    arm = arm_api.Arm(read_state=lambda: 'ok')
    arm._event = lambda task, kind, data: events.append(data)
    arm._pick_all_run = lambda *a, **k: {'ok': True, 'reason': None, 'reason_code': None, 'note': '새 물체 1개를 넣었습니다.'}
    arm._steady_scene = lambda grab: (None, scene)
    return arm, events


def test_pick_all_verifies_with_a_photo_after_success():
    arm, events = _arm(GOOD)
    with patch.dict(os.environ, {'ARM_TIDY_VERIFY': '1'}), patch.object(arm_api.CAMERA, 'release'):
        result = arm._pick_all('task', None)
    assert result['ok'] and result['verification']['ok']
    assert [e[0] for e in events] == ['verify_look', 'tidy_verify']
    assert result['note'].startswith('새 물체 1개를 넣었습니다. 정리 확인:')
    assert 'tidy_more' not in [e[0] for e in events]


def test_verify_photo_failure_keeps_success_and_failed_or_stopped_runs_skip_it():
    arm, events = _arm(GOOD)
    def broken(grab):
        raise OSError('Astra 없음')
    arm._steady_scene = broken
    with patch.dict(os.environ, {'ARM_TIDY_VERIFY': '1'}), patch.object(arm_api.CAMERA, 'release'):
        result = arm._pick_all('task', None)
        assert result['ok'] and '정리 확인 사진을 찍지 못했습니다' in result['note']
        arm._pick_all_run = lambda *a, **k: {'ok': False, 'reason_code': 'failed', 'reason': 'x'}
        assert 'verification' not in arm._pick_all('task', None)
        arm._pick_all_run = lambda *a, **k: {'ok': True}
        arm.stop_requested = True
        assert 'verification' not in arm._pick_all('task', None)


def _loop_arm(scenes, runs):
    events, queue = [], iter(scenes)
    arm = arm_api.Arm(read_state=lambda: 'ok')
    arm._event = lambda task, kind, data: events.append(data)
    arm._pick_all_run = lambda task, objects=None, include_new=False: runs.append((objects, include_new)) or {'ok': True, 'note': None}
    arm._steady_scene = lambda grab: (None, next(queue))
    return arm, events


LEFT = dict(GOOD, objects=[{'label': 'tape_measure', 'name': '줄자', 'pickable': True}], in_tray=GOOD['in_tray'][:2])


def test_unfinished_tidy_says_so_and_continues_with_what_is_left():
    runs = []
    arm, events = _loop_arm([LEFT, GOOD], runs)
    with patch.dict(os.environ, {'ARM_TIDY_VERIFY': '1'}), patch.object(arm_api.CAMERA, 'release'):
        result = arm._pick_all('task', ['red_cube', 'white_box', 'tape_measure'])
    assert runs == [(['red_cube', 'white_box', 'tape_measure'], False), (['tape_measure'], False)]      # 두 번째는 남은 것만
    assert [e[0] for e in events] == ['verify_look', 'tidy_verify', 'tidy_more', 'verify_look', 'tidy_verify']
    assert ('tidy_more', '책상 정리가 덜 됐습니다, 마저 정리하겠습니다.') in events
    assert result['ok'] and result['verification']['ok'] and result['note'].count('정리 확인:') == 1


def test_more_tidying_is_bounded_and_ignores_unreachable_leftovers():
    runs = []
    arm, events = _loop_arm([LEFT] * 6, runs)                     # 계속 남아 있다: 줄지 않으면 되풀이하지 않는다
    with patch.dict(os.environ, {'ARM_TIDY_VERIFY': '1'}), patch.object(arm_api.CAMERA, 'release'):
        result = arm._pick_all('task', None)
    assert len(runs) == 2 and not result['verification']['ok']
    runs.clear()
    far = dict(GOOD, objects=[{'label': 'tape_measure', 'name': '줄자', 'pickable': False, 'range': 'near', 'distance_mm': 120}])
    arm, events = _loop_arm([far], runs)
    with patch.dict(os.environ, {'ARM_TIDY_VERIFY': '1'}), patch.object(arm_api.CAMERA, 'release'):
        arm._pick_all('task', None)
    assert len(runs) == 1 and 'tidy_more' not in [e[0] for e in events]          # 닿지 않는 물체는 마저 하지 않는다


def test_failure_while_finishing_is_reported():
    runs = []
    arm, events = _loop_arm([LEFT], runs)
    results = iter([{'ok': True, 'note': None}, {'ok': False, 'reason': 'x', 'reason_code': 'failed'}])
    arm._pick_all_run = lambda *a, **k: next(results)
    with patch.dict(os.environ, {'ARM_TIDY_VERIFY': '1'}), patch.object(arm_api.CAMERA, 'release'):
        result = arm._pick_all('task', None)
    assert not result['ok'] and result['reason_code'] == 'failed'
