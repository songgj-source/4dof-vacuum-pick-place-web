"""새 물체를 비어 있는 칸에 배정한다: 가장 멀리 보낼 수 있는 물체가 가장 먼 빈 칸, 나머지는 고정 시드 무작위.

기존 물체(정육면체·흰 상자·줄자·등록 물체)는 자기 기본 칸에 넣고, 새 물체(unknown)는 남은 빈 칸을 채운다. 책상에는 항상 물체가 3개라서
기존 물체가 N개면 빈 칸은 3-N개이고 새 물체도 3-N개다(어긋나면 남는 물체를 알린다).

'가장 멀리 보낼 수 있다'는 말로 정하지 않고 놓기 계획기(place_plan.plan_place)로 잰다: 물체의 높이·반경으로 칸을 로봇 바깥쪽으로 옮겨 가며
계획이 서는 가장 먼 반경(reach_limit_mm)이다. 낮고 작은 물체일수록 멀리 간다. 하드웨어 접근 없음.
"""
import hashlib
import math
import random

from sorting import place_plan

REACH_SEARCH_LOW_M = .20          # 이 반경에서도 계획이 안 서면 그 물체는 어느 칸에도 못 간다고 본다(기본 칸은 이보다 가깝지 않다)
BISECT_STEPS = 7                  # (상한-하한)/2^7 ≈ 1.6mm 분해능
HOLD_START_Z_M = .070             # 집은 뒤 놓기 시작 자세 높이(기존 연속 정리와 같은 낮은 호버)
RIM_HEIGHT_MM = 40.               # 트레이 벽 높이(칸 간 순위만 비교하므로 모든 물체에 같은 값)
CONTACT_OFFSET_MM = -3.5


def _plans(model, angle, radius_m, held_height_mm, held_radius_mm, floor_mm, max_tilt_deg):
    """반경 radius_m, 방향 angle(rad)의 가상 칸에 놓기 계획이 서는가."""
    from tray.tray_detect import drop_z_mm
    drop = (drop_z_mm(floor_mm, held_height_mm, CONTACT_OFFSET_MM) + 20.) / 1000.      # sort_job.DROP_RAISE_MM과 같다
    slot = dict(x=radius_m * math.cos(angle), y=radius_m * math.sin(angle), drop_z=drop)
    start_r = min(max(.22, place_plan.START_RADIUS_RANGE[0] + .01), place_plan.START_RADIUS_RANGE[1] - .01)
    start = (start_r * math.cos(angle), start_r * math.sin(angle), HOLD_START_Z_M)
    need = place_plan.min_carry_z(held_height_mm, RIM_HEIGHT_MM, contact_offset_mm=CONTACT_OFFSET_MM)
    try:
        return place_plan.plan_place(model, slot, start, min_carry_z=need, held_height_mm=held_height_mm,
                                     contact_offset_mm=CONTACT_OFFSET_MM, held_radius_mm=held_radius_mm,
                                     max_tilt_deg=max_tilt_deg) is not None
    except ValueError:
        return False


def reach_limit_mm(model, angle, *, held_height_mm, held_radius_mm=place_plan.FOOTPRINT_RADIUS_M * 1000., floor_mm=10.,
                   max_tilt_deg=place_plan.DEFAULT_MAX_TILT_DEG):
    """이 물체를 들고 angle 방향으로 보낼 수 있는 가장 먼 칸 반경(mm). 가까운 곳에서도 안 되면 None."""
    lo, hi = REACH_SEARCH_LOW_M, place_plan.SLOT_RADIUS_MAX_M
    args = (model, angle)
    tail = (held_height_mm, held_radius_mm, floor_mm, max_tilt_deg)
    if not _plans(*args, lo, *tail):
        return None
    if _plans(*args, hi, *tail):
        return round(hi * 1000., 1)
    for _ in range(BISECT_STEPS):
        mid = (lo + hi) / 2.
        lo, hi = (mid, hi) if _plans(*args, mid, *tail) else (lo, mid)
    return round(lo * 1000., 1)


def assign(reach, slot_radius_mm, free, seed_text=''):
    """reach: {물체: reach_limit_mm 또는 None}, slot_radius_mm: {칸: 반경 mm}, free: 비어 있는 칸 번호들.
    돌려주는 것: ({물체: 칸}, 남는 물체 목록).
    가장 먼 빈 칸에는 거기까지 닿는 물체 중 reach가 가장 큰 것을 넣고, 나머지는 시드 고정 무작위로 닿는 칸에 하나씩 넣는다.
    같은 물체 조합·같은 빈 칸이면 항상 같은 결과다(시드 = 물체 이름과 빈 칸의 해시)."""
    keys = sorted(reach)
    free = sorted(free)
    digest = hashlib.md5(('|'.join(map(str, keys)) + '/' + ','.join(map(str, free)) + '/' + seed_text).encode()).hexdigest()
    rng = random.Random(int(digest[:12], 16))
    slots = {}
    # 가장 먼 '닿는' 빈 칸: 어떤 물체도 닿지 못하는 칸(예: 수직으로만 놓는 새 물체의 1번 칸)은 건너뛴다
    reachable_free = [n for n in free if any(reach[k] is not None and reach[k] >= slot_radius_mm[n] for k in keys)]
    if reachable_free:
        far = max(reachable_free, key=lambda n: slot_radius_mm[n])
        able = [k for k in keys if reach[k] is not None and reach[k] >= slot_radius_mm[far]]
        slots[max(able, key=lambda k: (reach[k], -keys.index(k)))] = far
    rest_keys = [k for k in keys if k not in slots]
    rest_slots = [n for n in free if n not in slots.values()]
    rng.shuffle(rest_keys)
    rng.shuffle(rest_slots)
    for key in rest_keys:
        for slot in rest_slots:
            if reach[key] is not None and reach[key] >= slot_radius_mm[slot]:
                slots[key] = slot
                rest_slots.remove(slot)
                break
    return slots, [k for k in keys if k not in slots]


def free_slots(in_tray_slots, default_slots_in_use, all_slots=(1, 2, 3)):
    """트레이에 이미 물체가 든 칸과 기존 물체가 쓸 기본 칸을 뺀 빈 칸."""
    taken = {int(s) for s in in_tray_slots} | {int(s) for s in default_slots_in_use}
    return [n for n in all_slots if n not in taken]
