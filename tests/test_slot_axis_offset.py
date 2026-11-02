"""2번 칸 목표를 트레이 길이 방향으로 로봇 쪽에 옮긴다(2026-11-02: 검출한 중심이 먼 쪽 칸막이 가장자리에 있었다)."""
import math

from sorting import sort_job


def test_axis_offset_moves_only_along_the_tray_axis_toward_the_robot():
    slots = {1: dict(x_mm=140., y_mm=-149.), 2: dict(x_mm=234.5, y_mm=-161.3), 3: dict(x_mm=353., y_mm=-173.)}
    axis = (353. - 140., -173. + 149.)
    norm = math.hypot(*axis)
    unit = (axis[0] / norm, axis[1] / norm)
    from tray.tray_capacity import slot_axis_xy
    got = slot_axis_xy(slots)
    assert math.isclose(got[0], unit[0]) and math.isclose(got[1], unit[1])
    assert sort_job.SLOT_AXIS_OFFSET_MM == {2: -25.}
    moved = (slots[2]['x_mm'] + unit[0] * -25., slots[2]['y_mm'] + unit[1] * -25.)
    along = (moved[0] - slots[2]['x_mm']) * unit[0] + (moved[1] - slots[2]['y_mm']) * unit[1]
    side = abs(-(moved[0] - slots[2]['x_mm']) * unit[1] + (moved[1] - slots[2]['y_mm']) * unit[0])
    assert math.isclose(along, -25.) and side < 1e-9 and moved[0] < slots[2]['x_mm']
