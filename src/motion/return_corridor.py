"""HOME 복귀를 시작해도 되는 자세 구간. 하드웨어 접근 없음.

기존: 높이 30~135mm, 반경 0.14~0.32m(pick_auto의 reset 구간과 같다).
추가: 반경 0.10~0.14m 띠는 높이 80~135mm일 때만(기존 run_hover_trial의 높은 호버 범위 안, 낮은 곳은 여전히 거부).
"""
import math


def start_in_corridor(xyz):
    x, y, z = (float(v) for v in xyz)
    if not all(math.isfinite(v) for v in (x, y, z)):
        return False
    r = math.hypot(x, y)
    if .030 <= z <= .135 and .14 <= r <= .32:
        return True
    return .080 <= z <= .135 and .10 <= r < .14


def interrupted_near_home_start(xyz, current_raw, home_raw):
    """Admit a stopped early approach to the existing sampled direct HOME path.

    This only selects the path. run_recorded_home_trial still checks every raw
    sample for controller margins and link clearance before sending any goal.
    """
    try:
        x, y, z = (float(v) for v in xyz)
        if not all(math.isfinite(v) for v in (x, y, z)) or not .080 <= z <= .250:
            return False
        if math.hypot(x, y) >= .14 or set(current_raw) != set(home_raw):
            return False
        if not current_raw or any(type(current_raw[n]) is not int or type(home_raw[n]) is not int for n in home_raw):
            return False
        return max(abs(current_raw[n] - home_raw[n]) for n in home_raw) <= 300
    except (TypeError, ValueError, KeyError):
        return False
