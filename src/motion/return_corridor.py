"""HOME 복귀를 시작해도 되는 자세 구간. 하드웨어 접근 없음.

기존: 높이 30~135mm, 반경 0.14~0.32m(pick_auto의 reset 구간과 같다).
추가: 반경 0.10~0.14m 띠는 높이 80~135mm일 때만(기존 run_hover_trial의 높은 호버 범위 안, 낮은 곳은 여전히 거부).
트레이 끝 복귀는 별도 선택: 기존 executor 반경·기울기 한도에서 현재 기울기를 유지해 먼저 올린다.
"""
import math

TRAY_START_RADIUS_MIN_M = .32
TRAY_START_HEIGHT_MIN_M = .030
TRAY_START_HEIGHT_MAX_M = .135


def start_in_corridor(xyz):
    x, y, z = (float(v) for v in xyz)
    if not all(math.isfinite(v) for v in (x, y, z)):
        return False
    r = math.hypot(x, y)
    if .030 <= z <= .135 and .14 <= r <= .32:
        return True
    return .080 <= z <= .135 and .10 <= r < .14


def corridor_diagnostics(xyz):
    """Explain the unchanged start gate; this does not authorize a path."""
    x, y, z = (float(v) for v in xyz)
    radius = math.hypot(x, y)
    finite = all(math.isfinite(v) for v in (x, y, z))
    inner = finite and radius < .14
    r_min, r_max = (.10, .14) if inner else (.14, .32)
    z_min, z_max = (.080, .135) if inner else (.030, .135)
    violations = []
    if not finite:
        violations.append('nonfinite_xyz')
    else:
        if radius < r_min: violations.append('radius_below_min')
        if radius > r_max: violations.append('radius_above_max')
        if z < z_min: violations.append('height_below_min')
        if z > z_max: violations.append('height_above_max')
    return dict(accepted=start_in_corridor((x, y, z)),
                radius_m=radius if math.isfinite(radius) else None,
                height_m=z if math.isfinite(z) else None,
                radius_bounds_m=[r_min, r_max], height_bounds_m=[z_min, z_max],
                violations=violations)


def tilted_tray_start(xyz, pitch_rad, *, radius_max_m, max_forward_tilt_rad):
    """Select a tilt-preserving lift at the existing place executor's edge.

    This is separate from the ordinary reset corridor and never selects a
    direct fold below 80 mm. The executor still checks the complete lift/fold
    paths, controller margins, held state, and tracking before/during motion.
    """
    try:
        x, y, z = (float(v) for v in xyz)
        pitch = float(pitch_rad)
        if not all(math.isfinite(v) for v in (x, y, z, pitch, radius_max_m, max_forward_tilt_rad)):
            return False
        return (TRAY_START_RADIUS_MIN_M < math.hypot(x, y) <= radius_max_m
                and TRAY_START_HEIGHT_MIN_M <= z <= TRAY_START_HEIGHT_MAX_M
                and -math.pi / 2 - 1e-9 <= pitch <= -math.pi / 2 + max_forward_tilt_rad + 1e-9)
    except (TypeError, ValueError):
        return False


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
