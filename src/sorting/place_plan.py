"""place(칸) 계획: 든 채로 칸 위로 옮겨 놓는 경유점과 도달 검사. 하드웨어 접근 없음.

계획만 만든다. 실행(든 채 이동)은 로봇을 쥔 프로세스 안에서 해야 하고, 사용자 승인과
실물 시험이 필요하다. 그래서 결과에는 항상 executed=False, usable_for_motion=False를 붙인다.
"""
import math

from sorting.carry_path import polar_path
from robot.robot_model import reachable
from motion.hover_trial import HOVER_RADIUS_MAX_M

ENVELOPE_M = (.16, .30)        # 집기에서 검증된 작업 거리
CARRY_Z = .130                 # 든 채 옮길 때 높이의 상한(m). 2026-10-20: 높이 41mm unknown이 2번 칸에 122mm가 필요해 120→130(IK·HOME 복귀 범위 0.135 안). 필요한 가장 낮은 높이만 쓴다
CARRY_STEP_M = .005            # 이동 높이를 낮춰 가며 찾는 간격
RIM_MARGIN_MM = 10.            # 물체 바닥과 트레이 벽 사이 여유(가정값)
OBSTACLE_MARGIN_MM = 10.       # 매단 물체 바닥과 다른 물체 윗면 사이 여유(가정값)
FOOTPRINT_RADIUS_M = .040      # 물체 하나의 수평 반경 가정값(실측 아님). 두 물체가 2배 안이면 겹칠 수 있다고 본다
DROP_Z_RANGE = (.015, .110)
MIN_CLEARANCE_M = .040         # 이동 높이는 놓는 높이보다 최소 이만큼 높아야 한다
START_Z_RANGE = (.030, .135)   # return_home.py 복귀 가능 구간과 같은 높이 범위
START_RADIUS_RANGE = (.14, .32)
VERTICAL = -math.pi / 2          # 공구가 수직 아래를 향한 pitch
DEFAULT_MAX_TILT_DEG = 60.       # 2026-10-21 사용자: 1번 칸 36.7cm가 50°로 닿지 않아 60°로(이전 50, 그 전 45). place 때 공구를 앞으로 기울여도 되는 한도(사용자 허용). 0이면 수직만. 필요한 가장 작은 기울기만 쓴다
TILT_STEP_DEG = 5.
MAX_TILT_DEG = 60.
SLOT_RADIUS_MAX_M = .38          # 기울이기를 허용하면 칸 반경은 이 한도까지 도달 가능성을 따져 본다(수직만이면 ENVELOPE_M[1])
CARRY_MARGIN_M = .010           # 운반 높이는 벽을 넘는 최소 높이(floor)보다 이만큼만 높게 시작한다(더 높으면 내리고 올리는 시간만 늘어난다)
TILT_MIN_Z = .070                # 기울이는 자리의 최소 높이(이동 코드의 낮은 호버 한계)


def min_carry_z(object_height_mm, rim_height_mm, *, margin_mm=RIM_MARGIN_MM, contact_offset_mm=-3.5):
    """든 물체가 트레이 벽을 넘으려면 필요한 컵 높이(m).

    물체는 컵 아래에 매달려 있고, 집을 때 컵은 물체 높이 + 접촉 보정(-3.5mm)에 있었다.
    따라서 물체 바닥 높이 = 컵 높이 - (물체 높이 + 접촉 보정). 이것이 벽 + 여유 이상이어야 한다.
    여유(margin_mm)는 가정값이다.
    """
    for name, value in (('object height', object_height_mm), ('rim height', rim_height_mm)):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be a positive finite number')
    if not math.isfinite(margin_mm) or margin_mm < 0 or not math.isfinite(contact_offset_mm):
        raise ValueError('margin must be a non-negative finite number')
    return (rim_height_mm + margin_mm + object_height_mm + contact_offset_mm) / 1000.


def _finite_slot(slot):
    if not isinstance(slot, dict):
        raise ValueError('slot dict with x, y, drop_z required')
    for key in ('x', 'y', 'drop_z'):
        value = slot.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'slot {key} must be a finite number')


def _radial_point(xy, scale):
    return (xy[0] * scale, xy[1] * scale)


def _segment_reachable(model, p0, p1, z, step=.01, pitch=VERTICAL):
    """p0→p1 수평 이동(실행과 같은 극좌표 경로) 위의 모든 점이 닿는가."""
    start = (p0[0], p0[1], z)
    end = (p1[0], p1[1], z)
    return reachable(model, start, pitch) and all(reachable(model, p, pitch) for p in polar_path(start, end, step))


UNTILT_STEP_DEG = 15.        # place_exec.MAX_TILT_STEP_RAD와 같은 값: 한 번에 15도 이하로 세운다
UNTILT_REACH_MARGIN_M = .006


def _retire_pitches(model, slot_xy, back, z, pitch):
    """후퇴(retire)하며 세우기 계획: 30mm 간격 경유점마다 쓸 pitch 목록(place_exec가 같은 경유점으로 실행한다).
    경유점마다 직전보다 15도 이하로만 줄이고, 그 구간 모든 지점(10mm 간격, 반경 6mm 여유 포함)이 그 pitch에서 닿는 가장 작은 기울기를 쓴다.
    먼 칸에서는 안쪽으로 충분히 들어온 뒤에야 세워진다(남은 기울기는 도착 뒤 제자리에서 세운다). 모든 지점이 모델로 검증된 pitch만 쓴다."""
    start = (slot_xy[0], slot_xy[1], z)
    waypoints = polar_path(start, (back[0], back[1], z), .030)
    tilt = math.degrees(pitch - VERTICAL)
    result, previous = [], start
    for point in waypoints:
        samples = [previous, *polar_path(previous, point, .01)]
        grown = [(q[0] * (math.hypot(q[0], q[1]) + UNTILT_REACH_MARGIN_M) / math.hypot(q[0], q[1]),
                  q[1] * (math.hypot(q[0], q[1]) + UNTILT_REACH_MARGIN_M) / math.hypot(q[0], q[1]), q[2]) for q in samples]
        chosen = None
        for candidate in [max(0., tilt - UNTILT_STEP_DEG) + k for k in range(int(math.ceil(min(UNTILT_STEP_DEG, tilt))) + 1)]:
            if candidate > tilt + 1e-9:
                break
            trial = VERTICAL + math.radians(candidate)
            if all(reachable(model, q, trial) for q in samples) and all(reachable(model, q, trial) for q in grown):
                chosen = candidate
                break
        if chosen is None:
            chosen = tilt          # 지금 기울기는 이 구간을 지나갈 수 있다(기존 계획이 검증한 기울기)
        tilt = chosen
        result.append(VERTICAL + math.radians(tilt))
        previous = point
    return result


def _carry_heights(floor, carry_z, lowest_z=0., rim_z=0.):
    """이동 높이 후보를 낮은 쪽부터(5mm 간격): floor(놓는 높이 + 40mm와 벽을 넘는 최소 높이 중 큰 값), 그리고 벽을 넘는 최소 높이(rim_z) + 10mm
    이상에서 닿는 가장 낮은 높이를 쓴다. 낮을수록 내리고 올리는 거리가 짧다.
    마지막 후보는 carry_z(상한). 닿는 곳이 없으면 호출자가 낮은 높이에서 막힌 이유를 알린다."""
    low = max(floor, rim_z + CARRY_MARGIN_M if rim_z else 0., lowest_z)
    low = math.ceil(low / CARRY_STEP_M - 1e-9) * CARRY_STEP_M
    heights, z = [], low
    while z < carry_z - 1e-9:
        heights.append(round(z, 4))
        z += CARRY_STEP_M
    heights.append(carry_z)
    return heights


def _distance_to_path(point, p0, p1):
    """장애물 점에서 극좌표 이동 경로(5mm 간격 표본)까지의 최소 거리."""
    start, end = (p0[0], p0[1], 0.), (p1[0], p1[1], 0.)
    return min(math.hypot(point[0] - q[0], point[1] - q[1]) for q in [start, *polar_path(start, end, .005)])


def _check_clearance(leg_name, p0, p1, cup_z, held_height_mm, contact_offset_mm, obstacles, held_radius_m=FOOTPRINT_RADIUS_M):
    """수평 이동 구간: 매단 물체 바닥이 가까운 물체 윗면 + 여유보다 높아야 한다."""
    bottom_mm = cup_z * 1000. - (held_height_mm + contact_offset_mm)
    for obstacle in obstacles:
        if _distance_to_path((obstacle['x'], obstacle['y']), p0, p1) < held_radius_m + obstacle.get('radius_mm', FOOTPRINT_RADIUS_M * 1000.) / 1000.:
            if bottom_mm < obstacle['height_mm'] + OBSTACLE_MARGIN_MM:
                raise ValueError(f"place leg {leg_name} could collide with {obstacle.get('label', 'an object')}: held object "
                                 f"bottom {bottom_mm:.0f} mm vs object top {obstacle['height_mm']:.0f} mm + "
                                 f"{OBSTACLE_MARGIN_MM:.0f} mm margin")


def _max_reachable_z(model, xy):
    z = .200
    while z > .005:
        if reachable(model, (xy[0], xy[1], z)):
            return z
        z -= CARRY_STEP_M
    return None


def plan_place(model, slot, start_xyz, *, carry_z=CARRY_Z, min_carry_z=None, held_height_mm=None,
               obstacles=(), contact_offset_mm=-3.5, held_radius_mm=FOOTPRINT_RADIUS_M * 1000.,
               max_tilt_deg=DEFAULT_MAX_TILT_DEG, start_tilt_deg=0.):
    """든 채로 칸에 놓는 계획(공구는 수직 아래 유지).

    1) 집은 자리에서 곧바로 이동 높이로 올라갈 수 있으면 그대로 간다(raise, carry, lower, retreat).
    2) 수직으로는 안 되면 공구를 앞으로 가장 조금만 기울여(최대 max_tilt_deg, 5도 단위) 곧바로 올라간다
       (raise_clear, tilt, raise, carry, lower, retreat, untilt). 기울이면 먼 거리에서도 더 높이 닿는다.
    3) 그래도 안 되고 거리가 멀면 그 높이에 닿지 않으면, 도달 가능한 높이에서 같은 방향으로 로봇 쪽으로 최소한만
       끌어당긴 뒤 올린다(raise_low, pull_in, raise, carry, lower, retreat).
    obstacles: 책상 위 다른 물체 [{label, x, y, height_mm}]. 끌어당기는 낮은 구간에서 부딪히면 거부한다.
    """
    _finite_slot(slot)
    if (isinstance(max_tilt_deg, bool) or not isinstance(max_tilt_deg, (int, float)) or not math.isfinite(max_tilt_deg)
            or not 0 <= max_tilt_deg <= MAX_TILT_DEG):
        raise ValueError(f'max tilt must be between 0 and {MAX_TILT_DEG:.0f} degrees')
    if (isinstance(start_tilt_deg, bool) or not isinstance(start_tilt_deg, (int, float)) or not math.isfinite(start_tilt_deg)
            or not 0 <= start_tilt_deg <= MAX_TILT_DEG):
        raise ValueError(f'start tilt must be between 0 and {MAX_TILT_DEG:.0f} degrees')
    # 먼 물체를 기울여 집었으면 팔이 이미 기울어 있다: 수직 계획은 건너뛰고, 기울기는 그 이상으로만 계획한다.
    start_pitch = VERTICAL + math.radians(start_tilt_deg)
    sx, sy, drop = float(slot['x']), float(slot['y']), float(slot['drop_z'])
    start = tuple(float(v) for v in start_xyz)
    if not all(math.isfinite(v) for v in start):
        raise ValueError('start pose must be finite')
    if not (START_Z_RANGE[0] <= start[2] <= START_Z_RANGE[1]
            and START_RADIUS_RANGE[0] <= math.hypot(*start[:2]) <= START_RADIUS_RANGE[1]):
        raise ValueError('start pose outside the vertical corridor')
    radius = math.hypot(sx, sy)
    max_tilt_deg = max(max_tilt_deg, start_tilt_deg)
    radius_max = SLOT_RADIUS_MAX_M if max_tilt_deg > 0 else ENVELOPE_M[1]
    if not ENVELOPE_M[0] <= radius <= radius_max:
        raise ValueError(f'tray slot radius {radius:.3f} m outside the {ENVELOPE_M[0]}-{radius_max} m pick envelope')
    if not DROP_Z_RANGE[0] <= drop <= DROP_Z_RANGE[1]:
        raise ValueError(f'drop z {drop*1000:.1f} mm outside {DROP_Z_RANGE[0]*1000:.0f}-{DROP_Z_RANGE[1]*1000:.0f} mm')
    if min_carry_z is not None and not math.isfinite(min_carry_z):
        raise ValueError('min_carry_z must be finite')
    floor = max(drop + MIN_CLEARANCE_M, min_carry_z or 0.)
    if floor > carry_z + 1e-9:
        raise ValueError(f'carry height {carry_z*1000:.0f} mm leaves less than the required clearance '
                         f'(needs {floor*1000:.0f} mm)')
    obstacles = list(obstacles)
    held_radius = float(held_radius_mm) / 1000.
    held = float(held_height_mm) if held_height_mm is not None else (40. if obstacles else None)
    checked = [o.get('label', 'object') for o in obstacles]
    slot_xy, start_xy = (sx, sy), start[:2]
    target_radius = radius

    def result(legs, z, pull_mm, tilt_deg=0.):
        if any(math.hypot(*item[1][:2]) > HOVER_RADIUS_MAX_M for item in legs):
            raise ValueError('planned tray move exceeds the hover radius before pickup')
        out = []
        for item in legs:
            leg = dict(name=item[0], target=item[1])
            if len(item) > 2:
                leg['pitch_rad'] = item[2]
            if len(item) > 3:
                leg.update(item[3])
            out.append(leg)
        return dict(legs=out, start=start, release_after='lower', carry_z=round(z, 4), tilt_deg=float(tilt_deg),
                    slot_radius_m=radius, target_radius_m=target_radius, object_outward_mm=round((target_radius - radius) * 1000., 1),
                    pull_in_mm=round(pull_mm, 1), obstacles_checked=checked,
                    assumed_footprint_radius_mm=FOOTPRINT_RADIUS_M * 1000, held_radius_mm=float(held_radius_mm), executed=False, usable_for_motion=False)

    # 1) 곧바로 올라가는 계획: 벽을 넘는 가장 낮은 이동 높이부터(낮을수록 내리고 올리는 시간이 짧다)
    blocked = None
    for z in (_carry_heights(floor, carry_z, rim_z=min_carry_z or 0.) if start_tilt_deg == 0 else ()):
        legs = [('raise', (*start_xy, z)), ('carry', (sx, sy, z)), ('lower', (sx, sy, drop)), ('retreat', (sx, sy, z))]
        if radius <= HOVER_RADIUS_MAX_M and all(reachable(model, point) for _, point in legs) and _segment_reachable(model, start_xy, slot_xy, z):
            try:
                if held is not None:
                    _check_clearance('carry', start_xy, slot_xy, z, held, contact_offset_mm, obstacles, held_radius)
            except ValueError as exc:
                blocked = blocked or exc          # 더 높은 이동 높이는 다른 물체 위를 넘을 수 있다
                continue
            return result(legs, z, 0.)
    if blocked is not None:
        raise blocked

    # 2) 공구를 앞으로 기울여 곧바로 올라가는 계획: 가장 작은 기울기부터(수직은 이미 위에서 시도했다)
    # 기울이면 컵 아래 매달린 물체 바닥이 컵보다 (물체 높이 × sin 기울기)만큼 바깥에 놓인다(흰 상자 24mm·50도: 18mm).
    # 칸 목표는 물체가 놓일 자리이므로 컵은 그만큼 안쪽으로 보낸다. 그래서 컵이 팔 한계에 있어도 물체는 더 멀리 놓인다
    # (2026-10-21 사용자 확인: 50도에서 실제로 약 2cm 더 멀리 놓임. 예전 1번 칸 고정 -20mm 보정을 대신한다).
    target_xy, target_radius = (sx, sy), radius
    z_clear = max(start[2], TILT_MIN_Z)
    if reachable(model, (*start_xy, z_clear), start_pitch):
        tilt = max(TILT_STEP_DEG, start_tilt_deg)
        while tilt <= max_tilt_deg + 1e-9:
            pitch = VERTICAL + math.radians(tilt)
            shift = (float(held_height_mm) if held_height_mm is not None else 0.) / 1000. * math.sin(math.radians(tilt))
            radius = target_radius - shift
            if radius > HOVER_RADIUS_MAX_M:
                tilt += TILT_STEP_DEG
                continue
            sx, sy = target_xy[0] * radius / target_radius, target_xy[1] * radius / target_radius
            slot_xy = (sx, sy)
            blocked = None
            for z in _carry_heights(floor, carry_z, z_clear, rim_z=min_carry_z or 0.):
                points = [(*start_xy, z_clear), (*start_xy, z), (sx, sy, z), (sx, sy, drop)]
                if all(reachable(model, p, pitch) for p in points) and _segment_reachable(model, start_xy, slot_xy, z, pitch=pitch):
                    try:
                        if held is not None:
                            _check_clearance('carry', start_xy, slot_xy, z, held, contact_offset_mm, obstacles, held_radius)
                    except ValueError as exc:
                        blocked = blocked or exc
                        continue
                    legs = []
                    if start[2] < z_clear - 1e-3:
                        legs.append(('raise_clear', (*start_xy, z_clear)) if start_tilt_deg == 0 else ('raise_clear', (*start_xy, z_clear), start_pitch))
                    if start_tilt_deg == 0 and z > z_clear + 2e-3 and reachable(model, (*start_xy, z)):
                        # 2026-10-20: 운반 높이까지 수직으로 먼저 올린 뒤 그 높이에서 기울인다. 70mm에서 기울이면 컵 끝이 상자 위 안전선(+25mm)에
                        # 가까워 15도씩 나눠야 해서(1번 칸 50도: 위로 빼기+기울이기 약 20초) 느렸다. 높은 곳은 여유가 커서 한 번에 기울일 수 있다.
                        legs += [('raise', (*start_xy, z)), ('tilt', (*start_xy, z), pitch)]
                    else:
                        legs.append(('tilt', (*start_xy, z_clear), pitch))
                        if z > z_clear + 2e-3:
                            legs.append(('raise', (*start_xy, z), pitch))
                    legs += [('carry', (sx, sy, z), pitch), ('lower', (sx, sy, drop), pitch), ('retreat', (sx, sy, z), pitch)]
                    if reachable(model, (sx, sy, z)):
                        legs.append(('untilt', (sx, sy, z), VERTICAL))
                    else:
                        # 칸에서는 수직으로 닿지 않는다(먼 칸): 기울인 채 안쪽으로 물러나 수직으로 닿는 곳에서 손목을 눕힌다.
                        # 그래야 HOME 복귀 시작 구간(반경 0.32m 이하)에 들어온다.
                        angle = math.atan2(sy, sx)
                        r_back = radius - CARRY_STEP_M
                        while r_back >= ENVELOPE_M[0]:
                            back = (r_back * math.cos(angle), r_back * math.sin(angle), z)
                            # 수직으로 닿는 한계(z=0.10m에서 반경 약 0.25m)의 경계에 서면 실행의 역기구학(관절 한계 여유 포함)이 해가 없다고 한다
                            # (2026-10-19: 반경 0.2497m): 후퇴 경유점과 같은 반경 여유(UNTILT_REACH_MARGIN_M)를 둔 곳에서도 닿는 곳만 쓴다.
                            grown_back = (back[0] * (r_back + UNTILT_REACH_MARGIN_M) / r_back,
                                          back[1] * (r_back + UNTILT_REACH_MARGIN_M) / r_back, z)
                            if reachable(model, back) and reachable(model, grown_back) and _segment_reachable(model, slot_xy, back[:2], z, pitch=pitch):
                                legs += [('retire', back, pitch, dict(retire_pitches=_retire_pitches(model, slot_xy, back, z, pitch))),
                                         ('untilt', back, VERTICAL)]
                                break
                            r_back -= CARRY_STEP_M
                    return result(legs, z, 0., tilt)
            if blocked is not None:
                raise blocked
            tilt += TILT_STEP_DEG
    (sx, sy), slot_xy, radius = target_xy, target_xy, target_radius

    # 3) 끌어당긴 뒤 올리는 계획: 필요한 최소 높이(5mm 단위 위로 올림)에서 가장 짧게 끌어당긴다
    steps = math.ceil(floor / CARRY_STEP_M - 1e-9)
    z = round(steps * CARRY_STEP_M, 4)
    r0 = math.hypot(*start_xy)
    top = _max_reachable_z(model, start_xy)
    if start_tilt_deg == 0 and z <= carry_z + 1e-9 and top is not None:
        z_low = round(math.floor((top - CARRY_STEP_M) / CARRY_STEP_M + 1e-9) * CARRY_STEP_M, 4)
        if z_low >= start[2] - 1e-9:
            distance = CARRY_STEP_M
            while r0 - distance >= ENVELOPE_M[0]:
                inner = _radial_point(start_xy, (r0 - distance) / r0)
                if (reachable(model, (*inner, z)) and _segment_reachable(model, start_xy, inner, z_low)
                        and _segment_reachable(model, inner, slot_xy, z)
                        and all(reachable(model, p) for p in ((*slot_xy, drop), (*slot_xy, z)))):
                    if held is not None:
                        _check_clearance('pull_in', start_xy, inner, z_low, held, contact_offset_mm, obstacles, held_radius)
                        _check_clearance('carry', inner, slot_xy, z, held, contact_offset_mm, obstacles, held_radius)
                    legs = [('raise_low', (*start_xy, z_low)), ('pull_in', (*inner, z_low)), ('raise', (*inner, z)),
                            ('carry', (sx, sy, z)), ('lower', (sx, sy, drop)), ('retreat', (sx, sy, z))]
                    return result(legs, z, distance * 1000.)
                distance += CARRY_STEP_M

    # 4) 어디서도 안 된다: 막힌 다리를 알린다
    for name, point in (('raise', (*start_xy, floor)), ('carry', (sx, sy, floor)), ('lower', (sx, sy, drop))):
        if not reachable(model, point):
            raise ValueError(f'place leg {name} not reachable at the lowest allowed carry height: '
                             f'{tuple(round(v, 3) for v in point)}')
    raise ValueError(f'place leg pull_in not reachable: no way to rise to {floor*1000:.0f} mm '
                     f'between radius {r0:.3f} m and {ENVELOPE_M[0]} m with the tool pointing down')
