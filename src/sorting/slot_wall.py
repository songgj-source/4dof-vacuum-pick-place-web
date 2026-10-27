"""먼 칸에서 물체를 칸 중심 대신 로봇 쪽 벽 가까이 놓기: 물체 방향 측정과 칸 안에서 당길 수 있는 거리. 하드웨어 접근 없음.

사용자 요청(2026-10-27): 팔이 크게 뻗어야 하는 칸(수직으로 곧장 닿지 않는 칸)은 로봇 쪽 벽에 붙여 넣어 덜 뻗게 한다.
4축 팔은 물체를 돌리지 못하므로, 칸에서의 물체 방향 = 책상에서 잰 방향 + 베이스가 돈 각도다.
방향을 모르면 물체를 대각선 원으로 봐야 해서 흰 상자는 약 4mm밖에 못 당긴다. 방향을 알면 사각형으로 따진다.
값은 계산 후보이며 실물로 확인하지 않았다(물체가 흡착 중 돌거나 미끄러지면 여유가 줄어든다).
"""
import math

AXIS_SIDE_TOLERANCE_MM = 20.     # 잰 변 길이와 등록 크기의 허용 차이. 넘으면 방향을 믿지 않는다(원으로 본다)
SQUARE_DIFF_MM = 10.             # 등록 가로·세로 차이가 이보다 작으면 어느 변이 긴지 가리지 않는다(긴 변 정사각형으로 본다)
MEASURED_DIFF_MM = 5.            # 잰 두 변의 차이가 이보다 작으면 긴 변을 고르지 않는다
WALL_MARGIN_MM = 6.              # 칸 벽과의 여유: 집기 위치 오차(약 ±3mm)와 놓을 때 흔들림(가정값)
YAW_ERROR_DEG = 6.               # 방향 측정·흡착 중 돌아감 여유(가정값): 이 범위의 가장 나쁜 각도로 따진다
MAX_INWARD_MM = 40.              # 계산이 어긋나도 이보다 멀리 당기지 않는다


def _unit(v):
    n = math.hypot(*v)
    if not math.isfinite(n) or n < 1e-9:
        raise ValueError('direction must be a finite non-zero vector')
    return (v[0] / n, v[1] / n)


def object_axis(corners_xy_mm, footprint_mm):
    """윗면 사각형 네 꼭짓점(로봇 xy, mm, 둘레 순서)과 등록 가로·세로(mm) → dict(axis_xy=긴 변 방향, half_mm=(긴, 짧은)/2).
    방향을 믿을 수 없으면 None(호출한 쪽은 대각선 원으로 본다)."""
    if not footprint_mm or len(corners_xy_mm) != 4:
        return None
    long_mm, short_mm = max(footprint_mm), min(footprint_mm)
    a = (corners_xy_mm[1][0] - corners_xy_mm[0][0], corners_xy_mm[1][1] - corners_xy_mm[0][1])
    b = (corners_xy_mm[2][0] - corners_xy_mm[1][0], corners_xy_mm[2][1] - corners_xy_mm[1][1])
    la, lb = math.hypot(*a), math.hypot(*b)
    if not all(math.isfinite(v) for v in (la, lb)) or min(la, lb) < 1.:
        return None
    (lm, vm), (ls, _) = sorted(((la, a), (lb, b)), key=lambda item: -item[0])
    if abs(lm - long_mm) > AXIS_SIDE_TOLERANCE_MM or abs(ls - short_mm) > AXIS_SIDE_TOLERANCE_MM:
        return None
    if long_mm - short_mm < SQUARE_DIFF_MM or lm - ls < MEASURED_DIFF_MM:
        # 정사각형에 가깝거나 잰 변으로 긴 쪽을 가릴 수 없다: 사각형 축 방향만 믿고 두 변 모두 긴 변으로 본다
        return dict(axis_xy=list(_unit(vm)), half_mm=[long_mm / 2., long_mm / 2.])
    return dict(axis_xy=list(_unit(vm)), half_mm=[long_mm / 2., short_mm / 2.])


def _extent(axis, half, direction):
    """방향(단위벡터)으로 잰 사각형의 중심~끝 거리."""
    other = (-axis[1], axis[0])
    return half[0] * abs(axis[0] * direction[0] + axis[1] * direction[1]) + half[1] * abs(other[0] * direction[0] + other[1] * direction[1])


def _rotate(v, angle):
    c, s = math.cos(angle), math.sin(angle)
    return (c * v[0] - s * v[1], s * v[0] + c * v[1])


def inward_limit_mm(slot_xy_mm, slot_axis_xy, inner_mm, pick_xy_mm, footprint=None, radius_mm=None,
                    margin_mm=WALL_MARGIN_MM, yaw_error_deg=YAW_ERROR_DEG):
    """칸 중심에서 로봇 쪽(반경 방향 안쪽)으로 물체를 옮겨도 칸 벽에 닿지 않는 최대 거리(mm, 0 이상).
    footprint: object_axis 결과(책상에서 잰 방향). 없으면 radius_mm(대각선 원)로 따진다."""
    u = _unit(slot_axis_xy)                     # 트레이 길이 방향(칸 안쪽 inner_mm[0])
    v = (-u[1], u[0])                           # 폭 방향(inner_mm[1])
    inward = _unit((-slot_xy_mm[0], -slot_xy_mm[1]))
    if footprint is not None:
        turn = math.atan2(slot_xy_mm[1], slot_xy_mm[0]) - math.atan2(pick_xy_mm[1], pick_xy_mm[0])
        axis = _unit(footprint['axis_xy'])
        half = footprint['half_mm']
        angles = [turn + math.radians(d) for d in (-yaw_error_deg, 0., yaw_error_deg)]
        ext_u = max(_extent(_rotate(axis, a), half, u) for a in angles)
        ext_v = max(_extent(_rotate(axis, a), half, v) for a in angles)
    else:
        if radius_mm is None or not math.isfinite(radius_mm) or radius_mm <= 0:
            return 0.
        ext_u = ext_v = float(radius_mm)
    limit = MAX_INWARD_MM
    for slack, d in ((inner_mm[0] / 2. - ext_u - margin_mm, inward[0] * u[0] + inward[1] * u[1]),
                     (inner_mm[1] / 2. - ext_v - margin_mm, inward[0] * v[0] + inward[1] * v[1])):
        if slack < 0:
            return 0.                           # 칸 중심에서도 여유가 없다: 당기지 않는다
        if abs(d) > 1e-6:
            limit = min(limit, slack / abs(d))
    return max(0., float(limit))
