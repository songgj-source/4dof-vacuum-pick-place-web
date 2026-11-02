"""새 물체가 너무 높으면(20~55mm 밖) 거리가 맞아도 못 넣는다: '빈 칸이 없어'가 아니라 높이가 이유라고 알린다(2026-11-02 높이 9.4cm 원통)."""
import sys
from pathlib import Path

sys.path[:0] = [str(Path(__file__).resolve().parents[1] / 'scripts'), str(Path(__file__).resolve().parents[1] / 'src')]
import arm_llm


def scene(**extra):
    return dict(objects=[dict(label='unknown', reachable=False, too_tall=True, height_mm=94.2, distance_mm=172, range='ok', pixel=[197, 307], **extra)], in_tray=[])


def test_reason_names_the_height_not_a_missing_slot():
    note = arm_llm.unreachable_note(scene())
    assert '너무 높습니다(9.4cm, 최대 5.5cm)' in note
    assert '옮겨 주세요' not in note                      # 높이는 옮겨도 그대로다
    assert '높아' in arm_llm.far_unknown_note(scene()) and '닿지' not in arm_llm.far_unknown_note(scene())


def test_distance_problems_still_ask_to_move_closer():
    far = dict(objects=[dict(label='unknown', reachable=False, range='far', distance_mm=360, pixel=[1, 1])], in_tray=[])
    note = arm_llm.unreachable_note(far)
    assert '너무 멉니다' in note and '16~30cm 안으로 옮겨 주세요' in note
    assert '팔이 닿지 않아' in arm_llm.far_unknown_note(far)


def test_check_report_after_tidy_gives_the_height_reason():
    ok, report = arm_llm.tidy_check_report(scene())
    assert not ok and '너무 높습니다' in report
