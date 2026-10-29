"""책상 정리에서 새 물체(등록 안 된 물체)를 기존 물체가 쓰고 남은 빈 칸에 넣는다. 책상에는 물체가 항상 3개다."""
import sys
from pathlib import Path
from unittest.mock import Mock, patch

sys.path[:0] = [str(Path(__file__).resolve().parents[1] / 'scripts'), str(Path(__file__).resolve().parents[1] / 'src')]
import arm_api

OK = {'ok': True, 'reason': None, 'reason_code': None}


def _arm(new_objects, placed, in_tray=(), pick=None, far=(), tray_found=True):
    arm = arm_api.Arm()
    arm.tasks['t'] = {}
    arm._event = Mock()
    arm._ensure_torque = lambda *a: None
    scene = dict(objects=[dict(label='unknown', pixel=[x, 240], height_mm=h, guess=g, reachable=x not in far) for x, h, g in new_objects],
                 in_tray=[dict(slot=s, label='red_cube') for s in in_tray], tray_found=tray_found,
                 slot_radius_mm={1: 344., 2: 253., 3: 168.})           # 실제 Pi 장면의 칸 거리(2026-10-28). 새 물체는 수직으로만 놓아 약 26cm까지
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


def test_new_objects_are_placed_vertically_so_slot_1_is_never_used():
    # 2026-10-28: 1번 칸(34cm)을 새 물체에 배정해 'unreachable(36.4cm, 최대 30cm)'로 멈췄다. 새 물체는 기울이지 않아 약 26cm까지만 닿는다
    arm = _arm([(200, 24., None), (300, 45., None)], placed=[2])                      # 기존 1개(2번 칸) + 새 2개: 빈 칸은 1·3번인데 1번은 못 간다
    result = _run(arm, ('white_box',))
    assert [c.args[2] for c in arm._pick.call_args_list] == [3] and 1 not in [c.args[2] for c in arm._pick.call_args_list]
    assert result['ok'] and '새 물체 1개를 넣었습니다' in result['note'] and '빈 칸이 없어 책상에 남겼습니다' in result['note']


def test_single_new_object_with_all_slots_looking_free_never_gets_slot_1():
    arm = _arm([(200, 28., None)], placed=[])                                          # 이번 실패 상황: 칸이 전부 빈 것으로 보였다
    _run(arm, ())
    slot = arm._pick.call_args.args[2]
    assert slot in (2, 3)


def test_three_new_objects_use_only_the_slots_they_can_reach():
    arm = _arm([(200, 30., None), (300, 35., None), (400, 40., None)], placed=[])
    result = _run(arm, ())                                                             # 기존 물체 없음(objects=[])
    arm._pick_registered.assert_not_called()
    assert sorted(call.args[2] for call in arm._pick.call_args_list) == [2, 3]         # 1번 칸은 못 가서 하나는 남는다
    assert '빈 칸이 없어 책상에 남겼습니다' in result['note']


def test_same_scene_gives_same_assignment():
    scene = [(200, 30., None), (300, 35., None), (400, 40., None)]
    first = _arm(scene, placed=[]); _run(first, ())
    second = _arm(scene, placed=[]); _run(second, ())
    assert first._pick.call_args_list == second._pick.call_args_list


def test_a_failed_new_object_stops_the_rest_and_says_which():
    arm = _arm([(200, 24., None), (300, 45., None)], placed=[2], pick=Mock(return_value=dict(ok=False, reason='suction failed', reason_code='x')))
    result = _run(arm, ('white_box',))
    assert not result['ok'] and '새 물체 1에서 멈췄습니다' in result['reason']
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


def test_unreachable_new_object_is_left_but_keeps_its_number_and_count():
    # 2026-10-28: 36.4cm(집기 한계 30cm)의 새 물체 때문에 정리 중간에 'unreachable'로 멈췄다. 닿지 않는 것은 미리 빼고 번호·개수는 유지한다
    arm = _arm([(200, 30., None), (300, 35., None)], placed=[1, 3], far=(200,))
    result = _run(arm, ('red_cube', 'tape_measure'))
    arm._pick.assert_called_once_with('t', 'unknown', 2, which=2, of=2)               # 닿는 오른쪽(2번)만, 개수는 닿지 않는 것도 포함(2)
    assert result['ok'] and '팔이 닿지 않아 남겼습니다' in result['note'] and '새 물체 1개를 넣었습니다' in result['note']


def test_only_unreachable_new_objects_means_nothing_is_picked():
    arm = _arm([(200, 30., None)], placed=[1, 3], far=(200,))
    result = _run(arm, ('red_cube', 'tape_measure'))
    arm._pick.assert_not_called()
    assert result['ok'] and '팔이 닿지 않아 책상에 남겼습니다' in result['note']


def test_describe_scene_marks_reachable_by_distance():
    import cv2
    import numpy as np
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tests'))
    root = Path(__file__).resolve().parents[1]
    rgb = cv2.imread(str(root / 'tests/data/unknown_20261028b/desk_rgb.png'))
    depth = np.load(root / 'tests/data/unknown_20261028b/desk_depth.npy')
    scene = arm_api.describe_scene(rgb, depth)
    unknown = [o for o in scene['objects'] if o['label'] == 'unknown'][0]
    # 실제 프레임의 갈색 물체는 약 16cm(집기 한계 16~30cm의 안쪽 끝). 카메라 변환이 바뀌면(2026-10-29 재보정 15.9cm) 경계 바로 옆이라 값이 넘나든다:
    # 거리를 읽어 한계와 비교하는 규칙만 확인한다
    assert 150 <= unknown['distance_mm'] <= 300
    assert unknown['reachable'] is (160 <= unknown['distance_mm'] <= 300)


def test_tray_not_found_uses_the_recently_seen_tray_or_refuses_to_guess():
    # 사진에서 트레이를 못 찾으면 '모든 칸이 빈 것'으로 보고 찬 칸에 넣으려 했다
    arm = _arm([(300, 30., None)], placed=[], tray_found=False)
    result = _run(arm, ())
    arm._pick.assert_not_called()
    assert not result['ok'] and result['reason_code'] == 'tray_unknown' and '트레이를 찾지 못해' in result['reason']
    arm = _arm([(300, 30., None)], placed=[], tray_found=False)
    arm.scene = dict(tray_found=True, in_tray=[dict(slot=2, label='white_box'), dict(slot=1, label='red_cube')])      # 직전에 본 트레이: 1·2번 칸이 찼다
    arm.scene_time = arm_api.time.monotonic()
    _run(arm, ())
    arm._pick.assert_called_once_with('t', 'unknown', 3, which=1, of=1)
    stale = _arm([(300, 30., None)], placed=[], tray_found=False)
    stale.scene = dict(tray_found=True, in_tray=[]); stale.scene_time = arm_api.time.monotonic() - 3600                  # 한 시간 전 기록은 믿지 않는다
    assert _run(stale, ())['reason_code'] == 'tray_unknown'
