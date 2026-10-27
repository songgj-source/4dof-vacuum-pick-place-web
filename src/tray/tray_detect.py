"""깊이로 3칸 트레이를 찾아 칸 1~3의 위치와 바닥 높이를 계산한다. 하드웨어 접근 없음.

트레이 벽(높이 55mm)과 칸막이를 깊이 높이로 찾고, 벽 윗가장자리 평면(물체와 같은 평면 투영)으로
로봇 좌표에 옮긴다. 칸막이 4개(양 끝 벽 2 + 사이 2)가 모두 보일 때만 칸을 돌려준다. 보이지 않으면
추측하지 않고 이유를 남긴다.

칸에 쓴 숫자는 이 해상도에서 읽을 수 없다. 번호는 긴 방향의 어느 끝이 1번인지 규칙(first_end)으로 정한다.
결과는 후보 계산이다(usable_for_motion=False, physical_accuracy_verified=False).
"""
import itertools
import math
from pathlib import Path

import cv2
import numpy as np

from perception.box_target import BoxLocator, TABLE_PLANE_Z
from perception.object_classify import MIN_HEIGHT_MM, classify_objects, table_height_mm, IGNORE_TOP_PX, _validate
from perception.object_targets import DEPTH_MINUS_TOP_CENTER_PX, REFERENCE_TOP_MM, _project
from tray.tray_digits import patch_at, read_slot_numbers_voted
from tray.tray_sorting import SLOT_FOR_LABEL

RIM_HEIGHT_MM = 55.
RIM_BAND_MM = (RIM_HEIGHT_MM - 17., RIM_HEIGHT_MM + 20.)   # 벽 높이 ±(깊이 오차·경사 가장자리)
RELEASE_GAP_MM = 5.            # 놓을 때 물체 바닥과 칸 바닥 사이(가정값)
MIN_RIM_PIXELS = 400
CLUSTER_DILATE_PX = 21
HISTOGRAM_BIN_MM = 3.
WALL_PEAK_FRACTION = .3        # 가장 큰 벽 피크 대비
WALL_MERGE_MM = 14.
COMPARTMENT_LENGTH_MM = (45., 200.)
TRAY_WIDTH_MM = (50., 220.)
FLOOR_WINDOW_PX = 8
FLOOR_BAND_MM = (4., 22.)       # 칸 바닥: 책상보다 약 10mm 높은 평평한 면
FLOOR_MIN_AREA_PX = 3000
FLOOR_MIN_SIDE_PX = 45
FLOOR_MAX_STD_MM = 4.
COMPARTMENT_PITCH_MM = (70., 125.)   # 인접 칸 중심 간격(트레이 길이 285mm 기준 약 95mm)
FLOOR_MIN_PIXELS = 40
FLOOR_MAX_HEIGHT_MM = 25.
FIRST_END_RULES = {
    'larger_y': lambda x, y: -y,
    'smaller_y': lambda x, y: y,
    'near_base': lambda x, y: math.hypot(x, y),
    'far_from_base': lambda x, y: -math.hypot(x, y),
}
READ_DIGITS = 'read_digits'    # 번호를 규칙이 아니라 칸에 쓴 숫자를 읽어서 정한다
SLOT_LABEL = {number: label for label, number in SLOT_FOR_LABEL.items()}


def _locator(root):
    base = Path(root) / 'output/ik_resume_20261012'
    return BoxLocator(base / 'multipose_review/camera_transform_candidate.json', base / 'moved_box_target.json')


def _rim_mask(bgr, depth, height):
    mask = (height >= RIM_BAND_MM[0]) & (height <= RIM_BAND_MM[1]) & (depth > 0)
    mask[:IGNORE_TOP_PX] = False
    # 물체(작은 덩어리)는 벽이 아니다. 트레이는 훨씬 커서 제외 대상이 아니다.
    for o in classify_objects(bgr, depth):
        x, y, w, h = o['bbox_px']
        if o['label'] != 'unknown' and max(w, h) < 110:
            mask[max(0, y - 6):y + h + 6, max(0, x - 6):x + w + 6] = False
    return mask


def _largest_cluster(mask):
    grown = cv2.dilate(mask.astype(np.uint8), np.ones((CLUSTER_DILATE_PX, CLUSTER_DILATE_PX), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(grown)
    best, best_pixels = 0, 0
    for i in range(1, count):
        pixels = int((mask & (labels == i)).sum())
        if pixels > best_pixels:
            best, best_pixels = i, pixels
    return (mask & (labels == best)) if best else np.zeros_like(mask)


def _wall_positions(t_values):
    """긴 방향 좌표(mm) 분포에서 벽 위치(피크 중심)를 찾는다."""
    lo, hi = float(t_values.min()), float(t_values.max())
    bins = max(1, int(math.ceil((hi - lo) / HISTOGRAM_BIN_MM)))
    counts, edges = np.histogram(t_values, bins=bins, range=(lo, lo + bins * HISTOGRAM_BIN_MM))
    centers = (edges[:-1] + edges[1:]) / 2
    strong = counts >= WALL_PEAK_FRACTION * counts.max()
    walls, group = [], []
    for center, count, ok in zip(centers, counts, strong):
        if ok and (not group or center - group[-1][0] <= WALL_MERGE_MM):
            group.append((center, count))
            continue
        if group:
            walls.append(group)
            group = []
        if ok:
            group.append((center, count))
    if group:
        walls.append(group)
    return [float(sum(c * n for c, n in g) / sum(n for _, n in g)) for g in walls]


def _color_pixel(locator, xy_m, height_mm):
    """로봇 xy(m)·책상 대비 높이(mm)의 점이 색 영상에 찍히는 픽셀. 없으면 None."""
    plane_z = TABLE_PLANE_Z - (REFERENCE_TOP_MM - height_mm) / 1000.
    world = np.array([xy_m[0] - locator.offset_xy[0], xy_m[1] - locator.offset_xy[1], plane_z])
    cam = locator.R @ world + locator.t
    if cam[2] <= 0:
        return None
    return locator.fx * cam[0] / cam[2] + locator.cx, locator.fy * cam[1] / cam[2] + locator.cy


def _floor_height(depth, height, locator, xy_m):
    """칸 중심(로봇 xy)을 바닥 평면으로 되투영한 깊이 픽셀 주변의 책상 대비 높이(mm). 모자라면 None."""
    plane_z = TABLE_PLANE_Z - REFERENCE_TOP_MM / 1000.
    world = np.array([xy_m[0] - locator.offset_xy[0], xy_m[1] - locator.offset_xy[1], plane_z])
    cam = locator.R @ world + locator.t
    if cam[2] <= 0:
        return None
    u = locator.fx * cam[0] / cam[2] + locator.cx + DEPTH_MINUS_TOP_CENTER_PX[0]
    v = locator.fy * cam[1] / cam[2] + locator.cy + DEPTH_MINUS_TOP_CENTER_PX[1]
    u0, v0 = int(round(u)), int(round(v))
    if not (FLOOR_WINDOW_PX <= u0 < 640 - FLOOR_WINDOW_PX and FLOOR_WINDOW_PX <= v0 < 480 - FLOOR_WINDOW_PX):
        return None
    window_h = height[v0 - FLOOR_WINDOW_PX:v0 + FLOOR_WINDOW_PX + 1, u0 - FLOOR_WINDOW_PX:u0 + FLOOR_WINDOW_PX + 1]
    window_d = depth[v0 - FLOOR_WINDOW_PX:v0 + FLOOR_WINDOW_PX + 1, u0 - FLOOR_WINDOW_PX:u0 + FLOOR_WINDOW_PX + 1]
    good = (window_d > 0) & (window_h < FLOOR_MAX_HEIGHT_MM)
    if int(good.sum()) < FLOOR_MIN_PIXELS:
        return None
    return float(max(0., np.median(window_h[good])))


TRAY_LENGTH_MM = 285.               # 트레이 바깥 길이(사용자 측정)
CARDBOARD_MIN_GAP = .20             # 두 가설의 끝 칸 갈색 비율 차가 이 이상이어야 색으로 가른다
WALL_LINE_UNCERTAINTY_MM = 15.      # 보이지 않는 끝 벽을 칸 간격으로 추정했을 때 그 칸 중심이 어긋날 수 있는 정도(가정값)


def _cardboard_fraction(bgr, pixels):
    """색 영상에서 네 꼭짓점(pixels)이 이루는 사각형 안의 골판지(갈색) 픽셀 비율. 꼭짓점이 없거나 영상 밖이면 0."""
    if any(p is None for p in pixels):
        return 0.
    polygon = np.round(np.array(pixels)).astype(np.int32)
    mask = np.zeros(bgr.shape[:2], np.uint8)
    cv2.fillPoly(mask, [polygon], 1)
    if not mask.any():
        return 0.
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    brown = (hsv[..., 0] >= 4) & (hsv[..., 0] <= 26) & (hsv[..., 1] >= 55) & (hsv[..., 1] <= 210) & (hsv[..., 2] >= 45) & (hsv[..., 2] <= 200)
    return float(brown[mask > 0].mean())


def _hidden_compartment_score(bgr, height, pixels):
    """가려진 끝 칸 가설 영역이 실제 트레이인 정도: 골판지 색 비율과 책상보다 높은(칸 바닥·물체) 비율 중 큰 값.
    칸에 노란 줄자처럼 갈색이 아닌 물체가 꽉 차 있어도 높이로 구분된다(2026-10-16 실물: 갈색 0.13 대 0.01로는 못 갈랐다)."""
    return max(_cardboard_fraction(bgr, pixels), _raised_fraction(height, pixels))


def _raised_fraction(height, pixels, min_mm=5.):
    """깊이 높이 영상에서 네 꼭짓점(색 영상 픽셀) 사각형 안이 책상보다 min_mm 이상 높은 비율. 트레이 칸은 바닥(약 10mm)이나
    안에 든 물체 때문에 높고, 트레이 밖 맨 책상은 0mm다. 색 픽셀을 깊이 픽셀로 옮겨(DEPTH_MINUS_TOP_CENTER_PX) 잰다."""
    if any(p is None for p in pixels):
        return 0.
    polygon = np.round(np.array(pixels) + np.array(DEPTH_MINUS_TOP_CENTER_PX)).astype(np.int32)
    mask = np.zeros(height.shape, np.uint8)
    cv2.fillPoly(mask, [polygon], 1)
    if not mask.any():
        return 0.
    return float((height[mask > 0] >= min_mm).mean())


def _complete_from_pair(walls, ok, cardboard_at):
    """벽이 2줄만 보일 때(칸 안 물체가 나머지를 가림): 두 줄이 1·2번, 2·3번, 3·4번 벽인 세 가설을 트레이 길이(285mm, 칸 3개를
    같은 길이로 가정)로 세우고, 세 칸 모두가 실제 트레이(골판지 색 또는 책상보다 높은 바닥·물체)인 정도의 합이 가장 큰 가설을 쓴다.
    1등과 2등의 차가 CARDBOARD_MIN_GAP보다 작거나 색·높이 정보가 없으면 추측하지 않는다. 반환: (벽 4개 또는 None, 추정한 벽 수)."""
    if cardboard_at is None:
        return None, 0
    pairs = [c for c in itertools.combinations(sorted(walls), 2) if ok(c)]
    if len(pairs) != 1:
        return None, 0
    p0, p1 = pairs[0]
    third = TRAY_LENGTH_MM / 3.
    scored = []
    for k in range(3):
        start = p0 - k * third
        lines = [start + i * third for i in range(4)]
        lines[k], lines[k + 1] = p0, p1
        scored.append((sum(cardboard_at(a, b) for a, b in zip(lines, lines[1:])), lines))
    scored.sort(key=lambda item: -item[0])
    if scored[0][0] - scored[1][0] < CARDBOARD_MIN_GAP:
        return None, 0
    return scored[0][1], 2


def _complete_walls(walls, t_values, cardboard_at=None):
    """벽 후보(긴 방향 좌표 mm)에서 4개를 고른다. 칸 안 물체가 만든 가짜 벽과 물체에 가려진 끝 벽을 다룬다.
    1) 간격이 칸 간격 범위(70~125mm)인 4개가 있으면 그것(여럿이면 간격이 가장 고른 것).
    2) 없고 간격이 맞는 3개 연속이면, 가려진 끝 벽이 어느 쪽인지 두 가설(앞/뒤에 트레이 길이 285mm만큼 보탬)을 세워서
       트레이 점군(t_values)이 더 많이 들어가는 쪽을 쓴다. 두 가설이 비슷하면(차이 10%p 미만) 추측하지 않는다.
       (2026-10-16: 예전에는 점군이 더 뻗은 쪽만 봐서, 물체 때문에 점군이 한쪽으로 늘어난 장면에서 칸이 한 칸 밀렸다.)
    반환: (벽 4개 또는 None, 추정한 벽 수)."""
    walls = sorted(walls)
    low, high = COMPARTMENT_PITCH_MM

    def ok(chain):
        return all(low <= b - a <= high for a, b in zip(chain, chain[1:]))
    for size in (4, 3):
        chains = [c for c in itertools.combinations(walls, size) if ok(c)]
        if chains:
            chain = min(chains, key=lambda c: float(np.std(np.diff(c))))
            break
    else:
        return _complete_from_pair(walls, ok, cardboard_at)
    if size == 4:
        return list(chain), 0
    t_values = np.asarray(t_values, float)

    def coverage(candidate):
        return float(np.mean((t_values >= candidate[0] - 12.) & (t_values <= candidate[-1] + 12.)))
    before, after = [chain[-1] - TRAY_LENGTH_MM, *chain], [*chain, chain[0] + TRAY_LENGTH_MM]
    # 1순위: 색 영상. 가려진 끝 칸이 실제로 골판지(갈색)인 쪽이 맞다(맨 책상 쪽은 갈색이 거의 없다).
    if cardboard_at is not None:
        b_score = cardboard_at(before[0], before[1])
        a_score = cardboard_at(after[-2], after[-1])
        if abs(a_score - b_score) >= CARDBOARD_MIN_GAP:
            return (before if b_score > a_score else after), 1
    # 2순위: 점군을 더 많이 감싸는 쪽. 비슷하면 추측하지 않는다.
    c_before, c_after = coverage(before), coverage(after)
    if abs(c_before - c_after) < .10:
        return None, 0
    return (before if c_before > c_after else after), 1


# 트레이 방향(긴 축): 깊이에는 벽 일부만 보일 수 있다(2026-10-19: 위쪽 1/3 조각). 조각의 PCA는 방향이 12° 틀어졌으므로
# 색 영상의 판지 실루엣(길이:폭 = 285:120이라 방향이 유일)으로 먼저 정하고, 칸막이 정렬로 다듬는다. 트레이는 아무 각도에나 놓일 수 있다.
CARDBOARD_SAT_MIN = 55          # 판지 채도 80~180, 흰 책상은 25 이하
CARDBOARD_VALUE = (28, 150)     # 판지 명도 30~100(검은 선·로봇보다 밝고 책상보다 어둡다)
CARDBOARD_HUE_MAX = 25
CARDBOARD_MIN_AREA_PX = 6000
OUTLINE_MIN_ASPECT = 1.8
OUTLINE_MATCH_MARGIN_PX = 15
OUTLINE_REFINE_WINDOW_DEG = 10.
OUTLINE_REFINE_STEP_DEG = .25
REFINE_MIN_GAIN = 1.25
PCA_MIN_ASPECT = 1.8            # 색 실루엣이 없을 때, 이보다 둥근 벽 조각의 PCA 방향은 믿지 않는다
PCA_TRUST_ASPECT = 2.0          # 색 실루엣과 함께 있을 때 PCA를 그대로 쓰는 최소 길쭉함
PCA_OUTLINE_AGREE_DEG = 15.     # PCA와 실루엣 방향이 이보다 다르면 실루엣을 따른다(조각 PCA가 90°까지 틀릴 수 있다)


def _cardboard_blob(bgr):
    """색 영상에서 가장 큰 판지색 덩어리(마스크, 윤곽). 없으면 None."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, sat, val = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    mask = (((h <= CARDBOARD_HUE_MAX) | (h >= 170)) & (sat >= CARDBOARD_SAT_MIN)
            & (val >= CARDBOARD_VALUE[0]) & (val <= CARDBOARD_VALUE[1])).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    if count < 2:
        return None
    best = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[best, cv2.CC_STAT_AREA] < CARDBOARD_MIN_AREA_PX:
        return None
    contours, _ = cv2.findContours((labels == best).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour = max(contours, key=cv2.contourArea)
    filled = np.zeros(mask.shape, np.uint8)
    cv2.drawContours(filled, [contour], -1, 1, thickness=cv2.FILLED)     # 옅은 벽 윗면·그림자·칸 안쪽까지 실루엣 안에 넣는다
    return filled, contour


def _tray_axis_from_color(bgr):
    """판지 실루엣의 긴 변 방향(색 영상 픽셀 좌표 단위벡터)과 중심·길이:폭 비. 길쭉하지 않으면 None."""
    found = _cardboard_blob(bgr)
    if found is None:
        return None
    blob, contour = found
    (cx, cy), (w, h), _ = cv2.minAreaRect(contour)
    if min(w, h) < 1 or max(w, h) / min(w, h) < OUTLINE_MIN_ASPECT:
        return None
    box = cv2.boxPoints(cv2.minAreaRect(contour))
    edges = [box[(i + 1) % 4] - box[i] for i in range(4)]
    edge = max(edges, key=np.linalg.norm)
    direction = edge / np.linalg.norm(edge)
    return dict(direction=[float(direction[0]), float(direction[1])], center=[float(cx), float(cy)],
                length_px=float(max(w, h)), aspect=float(max(w, h) / min(w, h)), blob=blob)


def _refine_long_axis(points, center, guess, window_deg=OUTLINE_REFINE_WINDOW_DEG, step_deg=OUTLINE_REFINE_STEP_DEG):
    """guess(단위벡터) 근처에서, 가로 벽(끝 벽·칸막이)이 긴 방향 히스토그램에 가장 날카롭게 모이는 방향을 고른다."""
    base = math.atan2(guess[1], guess[0])
    guess = np.asarray(guess, float)
    best, best_score, guess_score = guess, -1., None
    rel = points - center
    for delta in np.arange(-window_deg, window_deg + step_deg / 2, step_deg):
        angle = base + math.radians(delta)
        axis = np.array([math.cos(angle), math.sin(angle)])
        t = rel @ axis
        counts, _ = np.histogram(t, bins=max(1, int(math.ceil((t.max() - t.min()) / HISTOGRAM_BIN_MM))))
        score = float(np.sum(counts.astype(float) ** 2))
        if abs(delta) < step_deg / 2:
            guess_score = score
        if score > best_score:
            best, best_score = axis, score
    # 조각처럼 벽 정렬 정보가 약하면 봉우리가 완만하다(2026-10-19 실물 조각: 개선 5%, 오히려 6° 틀어짐). 뚜렷할 때만 색 실루엣 방향을 바꾼다.
    if guess_score is None or best_score < REFINE_MIN_GAIN * guess_score:
        return guess
    return best


def _long_axis_from_outline(bgr, locator, rim_height_mm, cluster_mask_color_px):
    """색 실루엣의 긴 축을 로봇 xy 평면 단위벡터로. 실루엣이 벽 덩어리를 포함하지 않으면 None(다른 물체일 수 있다)."""
    outline = _tray_axis_from_color(bgr)
    if outline is None:
        return None
    ys, xs = cluster_mask_color_px
    # 깊이↔색 정합 오차(몇 px)를 허용하려고 실루엣을 조금 부풀려서 벽 덩어리가 그 안에 있는지 본다.
    grown = cv2.dilate(outline['blob'], np.ones((OUTLINE_MATCH_MARGIN_PX, OUTLINE_MATCH_MARGIN_PX), np.uint8))
    inside = grown[np.clip(np.round(ys).astype(int), 0, 479), np.clip(np.round(xs).astype(int), 0, 639)]
    if float(inside.mean()) < .8:
        return None
    plane_z = TABLE_PLANE_Z - (REFERENCE_TOP_MM - rim_height_mm) / 1000.
    c = np.array(outline['center'])
    d = np.array(outline['direction'])
    half = outline['length_px'] / 2
    a = _project(locator, tuple(c - d * half), plane_z) + locator.offset_xy
    b = _project(locator, tuple(c + d * half), plane_z) + locator.offset_xy
    v = (b - a) * 1000.
    return v / np.linalg.norm(v)



LAYOUT_PITCH_MM = (80., 110.)       # 인접 칸 중심 간격(길이 285mm 트레이 약 95mm). 칸 안 물체 윗면이 벽으로 보이면 이 범위를 벗어난다.
LAYOUT_PITCH_DIFF_MM = 20.          # 두 간격의 차이(벽 추정 불확실도 20mm 이내)


def _layout_problem(compartments):
    """칸 3개 중심이 일정한 간격으로 한 줄에 놓였는지. 아니면 이유(문자열), 맞으면 None. 목록 순서와 무관하다."""
    pts = np.array([np.asarray(c, float) for c in compartments])
    axis = pts[-1] - pts[0]
    if np.linalg.norm(axis) < 1e-6:
        return 'compartment centres coincide'
    axis /= np.linalg.norm(axis)
    order = np.argsort(pts @ axis)
    pts = pts[order]
    pitches = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    if np.any(pitches < LAYOUT_PITCH_MM[0]) or np.any(pitches > LAYOUT_PITCH_MM[1]):
        return (f'compartment spacing {pitches[0]:.0f}/{pitches[1]:.0f} mm is outside {LAYOUT_PITCH_MM[0]:.0f}-{LAYOUT_PITCH_MM[1]:.0f} mm; '
                'an object in a compartment may have been taken for a wall')
    if abs(pitches[0] - pitches[1]) > LAYOUT_PITCH_DIFF_MM:
        return (f'compartment spacing {pitches[0]:.0f}/{pitches[1]:.0f} mm is uneven; '
                'an object in a compartment may have been taken for a wall')
    return None



def _by_walls(bgr, depth, height, locator, rim_height_mm):
    """벽(높이 55mm)과 칸막이로 칸 3개를 찾는다. (이유, 칸 중심들, 바닥 높이들, 추가 정보)."""
    cluster = _largest_cluster(_rim_mask(bgr, depth, height))
    if int(cluster.sum()) < MIN_RIM_PIXELS:
        return 'no tray walls found: too few wall-height pixels', None, None, None
    ys, xs = np.nonzero(cluster)
    plane_z = TABLE_PLANE_Z - (REFERENCE_TOP_MM - rim_height_mm) / 1000.
    step = max(1, len(xs) // 4000)
    points = np.array([(_project(locator, (x - DEPTH_MINUS_TOP_CENTER_PX[0], y - DEPTH_MINUS_TOP_CENTER_PX[1]), plane_z)
                        + locator.offset_xy) * 1000. for x, y in zip(xs[::step], ys[::step])])
    center = points.mean(axis=0)
    eigenvalues, axes = np.linalg.eigh(np.cov((points - center).T))
    long_axis = axes[:, 1]
    color_px = (ys[::step] - DEPTH_MINUS_TOP_CENTER_PX[1], xs[::step] - DEPTH_MINUS_TOP_CENTER_PX[0])
    outlined = _long_axis_from_outline(bgr, locator, rim_height_mm, color_px)
    pca_aspect = math.sqrt(eigenvalues[1] / max(eigenvalues[0], 1e-9))
    pca_agrees = (outlined is not None
                  and abs(float(outlined @ long_axis)) >= math.cos(math.radians(PCA_OUTLINE_AGREE_DEG)))
    if outlined is None:
        if pca_aspect < PCA_MIN_ASPECT:
            return ('wall fragment is too compact to give the tray direction and no tray outline was found in the colour image; '
                    'refusing to guess the direction'), None, None, None
        direction_source = 'wall_pca'
    elif pca_aspect >= PCA_TRUST_ASPECT and pca_agrees:
        direction_source = 'wall_pca'      # 트레이 대부분이 깊이에 보이면 벽 덩어리 PCA가 가장 정확하다(색 실루엣은 원근으로 수도 어긋난다)
    else:
        if outlined @ long_axis < 0:
            outlined = -outlined
        long_axis = _refine_long_axis(points, center, outlined)
        direction_source = 'cardboard_outline'
    cross_axis = np.array([-long_axis[1], long_axis[0]])
    t = (points - center) @ long_axis
    a = (points - center) @ cross_axis
    a_lo, a_hi = np.percentile(a, [2, 98])
    a_mid, a_half = (a_lo + a_hi) / 2, (a_hi - a_lo) / 2
    if not TRAY_WIDTH_MM[0] <= 2 * a_half <= TRAY_WIDTH_MM[1]:
        return f'wall cluster width {2 * a_half:.0f} mm does not look like the tray', None, None, None
    inner = np.abs(a - a_mid) < .8 * a_half            # 옆 벽은 빼고 가로지르는 벽만 센다
    if int(inner.sum()) < 50:
        return 'no cross walls found: could not see the compartment dividers', None, None, None
    found_walls = _wall_positions(t[inner])
    strict = (len(found_walls) == 4 and not np.any(np.diff(found_walls) < COMPARTMENT_LENGTH_MM[0])
              and not np.any(np.diff(found_walls) > COMPARTMENT_LENGTH_MM[1]))
    walls, inferred = (found_walls, 0) if strict else _complete_walls(
        found_walls, t, lambda t0, t1: _hidden_compartment_score(bgr, height, [
            _color_pixel(locator, (center + tt * long_axis + (a_mid + side * a_half) * cross_axis) / 1000., rim_height_mm)
            for tt, side in ((t0, -1), (t0, 1), (t1, 1), (t1, -1))]))
    if walls is not None and inferred:
        # 가려진 끝 벽이 색 영상 안에 있을 때만 추정한다(영상 밖으로 잘린 트레이는 예전처럼 거부).
        for hidden in [w for w in walls if all(abs(w - f) > 1e-6 for f in found_walls)]:
            px = _color_pixel(locator, (center + hidden * long_axis + a_mid * cross_axis) / 1000., rim_height_mm)
            if px is None or not (20 <= px[0] <= 620 and 20 <= px[1] <= 460):
                walls = None
                break
    if walls is None:
        return (f'found {len(found_walls)} wall lines along the tray, need 4 (two ends and two dividers) '
                'to tell the 3 compartments apart; part of the tray may be outside the depth view'), None, None, None
    lengths = np.diff(walls)
    if np.any(lengths < COMPARTMENT_LENGTH_MM[0]) or np.any(lengths > COMPARTMENT_LENGTH_MM[1]):
        return ('compartment lengths ' + ', '.join(f'{v:.0f}' for v in lengths)
                + f' mm are outside {COMPARTMENT_LENGTH_MM[0]:.0f}-{COMPARTMENT_LENGTH_MM[1]:.0f} mm; refusing to guess'), None, None, None
    compartments = [center + ((lo + hi) / 2) * long_axis + a_mid * cross_axis for lo, hi in zip(walls[:-1], walls[1:])]
    if (problem := _layout_problem(compartments)) is not None:
        return problem, None, None, None
    floors = [_floor_height(depth, height, locator, xy / 1000.) for xy in compartments]
    return None, compartments, floors, dict(tray_length_mm=float(walls[-1] - walls[0]), tray_width_mm=float(2 * a_half),
                                            long_axis=[float(v) for v in long_axis], inferred_wall_lines=inferred,
                                            direction_source=direction_source)


def _by_floors(depth, height, locator):
    """칸 바닥 3개(책상보다 약 10mm 높은 넓고 평평한 면)로 칸을 찾는다. 벽 옆면 깊이가 없을 때 쓴다."""
    mask = ((height >= FLOOR_BAND_MM[0]) & (height <= FLOOR_BAND_MM[1]) & (depth > 0)).astype(np.uint8)
    mask[:IGNORE_TOP_PX] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
    blobs = []
    for i in range(1, count):
        x, y, w, h, area = (int(v) for v in stats[i])
        if area < FLOOR_MIN_AREA_PX or min(w, h) < FLOOR_MIN_SIDE_PX or area < .5 * w * h:
            continue
        values = height[labels == i]
        if float(np.std(values)) > FLOOR_MAX_STD_MM:
            continue
        pixel = np.array(centroids[i]) - np.array(DEPTH_MINUS_TOP_CENTER_PX)
        floor = float(np.median(values))
        plane_z = TABLE_PLANE_Z - (REFERENCE_TOP_MM - floor) / 1000.
        blobs.append(dict(area=area, floor=floor, xy=(_project(locator, pixel, plane_z) + locator.offset_xy) * 1000.))
    # 물체·책상 얼룩이 섞여도 되도록, 칸 3개의 조건(같은 간격, 일직선, 비슷한 크기)을 모두 만족하는 세 개 조합만 인정한다.
    triples = []
    for trio in itertools.combinations(blobs, 3):
        points = np.array([b['xy'] for b in trio])
        axis = np.linalg.eigh(np.cov((points - points.mean(axis=0)).T))[1][:, 1]
        ordered = [trio[i] for i in np.argsort((points - points.mean(axis=0)) @ axis)]
        xy = np.array([b['xy'] for b in ordered])
        pitch = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        line = (xy[-1] - xy[0]) / np.linalg.norm(xy[-1] - xy[0])
        # 2D 외적(직선에서 떨어진 거리). numpy 2.x는 2차원 벡터의 np.cross를 지원하지 않는다.
        off = max(abs(float(line[0] * (p - xy[0])[1] - line[1] * (p - xy[0])[0])) for p in xy)
        if (np.all((pitch >= COMPARTMENT_PITCH_MM[0]) & (pitch <= COMPARTMENT_PITCH_MM[1])) and pitch.max() <= 1.4 * pitch.min()
                and off <= .2 * pitch.mean() and min(b['area'] for b in ordered) >= .3 * max(b['area'] for b in ordered)):
            triples.append((ordered, [float(v) for v in line]))
    if len(triples) != 1:
        return (f'found {len(blobs)} flat floor-like areas forming {len(triples)} possible 3-compartment rows, need exactly 1; '
                'the tray floors are not clearly visible in the depth view'), None, None, None
    blobs, line = triples[0]
    compartments = [b['xy'] for b in blobs]
    return None, compartments, [b['floor'] for b in blobs], dict(
        tray_length_mm=None, tray_width_mm=None, long_axis=line,
        floor_method_note='centres are the visible floor areas; walls were not visible in depth')


METHODS = ('auto', 'walls', 'floors')
FLOOR_METHOD_UNCERTAINTY_MM = 25.   # 끝 칸 바닥이 벽 그림자에 가려 중심이 치우치는 정도(벽 방식과 비교한 최대 차이 23mm)


WALLS_FLOORS_DISAGREE_MM = 30.


def _slots_on_cardboard(bgr, detection, root):
    """칸 중심 3개가 색 영상의 판지(트레이) 실루엣 안에 찍히는가. 판지를 못 찾으면 None(판단 안 함)."""
    found = _cardboard_blob(bgr)
    if found is None:
        return None
    blob = found[0]
    locator = _locator(root)
    for slot in detection['slots'].values():
        px = _color_pixel(locator, (slot['x_mm'] / 1000., slot['y_mm'] / 1000.), slot.get('floor_height_mm') or 10.)
        if px is None:
            return False
        u, v = int(round(px[0])), int(round(px[1]))
        if not (0 <= v < blob.shape[0] and 0 <= u < blob.shape[1]) or not blob[v, u]:
            return False
    return True


def detect_tray_checked(bgr, depth, root, *, prefer_floors=False, **kw):
    """트레이 인식에 두 겹 확인을 더한다(2026-10-27 사용자: 트레이 인식 오류가 종종 난다).

    Pi에 저장된 빈 트레이 사진 176장에서 칸 바닥 방식은 176장 모두 맞는 배치(간격·일직선)를 찾았고, 벽 방식은 20장에서만
    찾았는데 모두 안 보이는 벽선을 트레이 길이로 채운 추정이었고 그중 2장은 87mm 밀리거나 트레이를 90도 돌려 잡았다
    (2026-10-27: 칸 3이 책상 위 정육면체 26mm 옆에 찍혀 정육면체를 '칸 안'으로 빼고 흰 상자를 엉뚱한 곳에 놓았다).
    - prefer_floors: 빈 트레이로 저장한 기준 사진(칸 바닥 3개가 보여 저장됨)은 칸 바닥 방식을 먼저 쓴다.
    - 벽 방식이 벽선을 추정했으면 칸 바닥 방식과 비교해 30mm 넘게 다르면 칸 바닥 방식을 쓴다.
    - 칸 바닥이 안 보이면(칸에 물체) 추정한 칸 중심 3개가 모두 판지 실루엣 안에 있어야 쓴다. 아니면 찾지 못한 것으로 한다(엉뚱한 곳에 놓지 않게)."""
    if prefer_floors:
        floors = detect_tray(bgr, depth, root, **{**kw, 'method': 'floors'})
        if floors.get('found'):
            return floors
    found = detect_tray(bgr, depth, root, **kw)
    if not found.get('found') or found.get('method') != 'walls' or not any('hidden' in p for p in found.get('problems', [])):
        return found
    floors = detect_tray(bgr, depth, root, **{**kw, 'method': 'floors'})
    if floors.get('found'):
        gap = max(math.hypot(found['slots'][k]['x_mm'] - floors['slots'][k]['x_mm'], found['slots'][k]['y_mm'] - floors['slots'][k]['y_mm'])
                  for k in found['slots'] if k in floors['slots'])
        if gap <= WALLS_FLOORS_DISAGREE_MM:
            return found
        floors.setdefault('problems', []).append(f'wall method (inferred walls) disagreed with the floor method by {gap:.0f} mm; used the floors')
        return floors
    if _slots_on_cardboard(bgr, found, root) is False:
        return dict(found=False, first_end=found.get('first_end'), usable_for_motion=False, physical_accuracy_verified=False,
                    reason='tray layout not confirmed: the wall method had to guess hidden walls and its compartments fall outside '
                           'the cardboard tray in the colour image (move objects so the tray floors or walls are visible)')
    return found


def detect_tray(bgr, depth, root, *, first_end='larger_y', rim_height_mm=RIM_HEIGHT_MM, locator=None, extra_bgr=(),
                method='auto'):
    if first_end not in FIRST_END_RULES and first_end != READ_DIGITS:
        raise ValueError(f'first_end must be one of {sorted(FIRST_END_RULES) + [READ_DIGITS]}')
    if method not in METHODS:
        raise ValueError(f'method must be one of {METHODS}')
    _validate(bgr, depth)
    locator = locator or _locator(root)
    base = dict(found=False, first_end=first_end, usable_for_motion=False, physical_accuracy_verified=False)
    height = table_height_mm(depth)
    reasons, used = [], None
    for name in (('walls', 'floors') if method == 'auto' else (method,)):
        if name == 'walls':
            reason, compartments, floors, extras = _by_walls(bgr, depth, height, locator, rim_height_mm)
        else:
            reason, compartments, floors, extras = _by_floors(depth, height, locator)
        if reason is None:
            used = name
            break
        reasons.append(f'{name}: {reason}')
    if used is None:
        return dict(base, reason='; '.join(reasons))
    numbering = dict(numbering='rule:' + first_end)
    if first_end == READ_DIGITS:
        pixels = [_color_pixel(locator, xy / 1000., floor or 0.) for xy, floor in zip(compartments, floors)]
        triples = [[patch_at(frame, px) for px in pixels] for frame in (bgr, *extra_bgr)]
        if any(p is None for p in triples[0]):
            return dict(base, reason='could not read the compartment numbers: a compartment is outside the colour image')
        reading = read_slot_numbers_voted(triples)
        if not reading['found']:
            return dict(base, reason=reading['reason'], digits=reading)
        ranked = sorted(range(3), key=lambda i: reading['numbers'][i])
        numbering = dict(numbering=READ_DIGITS, digits=reading)
    else:
        key = FIRST_END_RULES[first_end]
        ranked = sorted(range(3), key=lambda i: key(compartments[i][0], compartments[i][1]))
    compartments = [compartments[i] for i in ranked]
    floors = [floors[i] for i in ranked]
    slots, problems = {}, []
    for number, (xy, floor) in enumerate(zip(compartments, floors), 1):
        if floor is None:
            problems.append(f'slot {number}: floor height not measured (too few valid depth pixels)')
        slots[number] = dict(x_mm=float(xy[0]), y_mm=float(xy[1]), floor_height_mm=floor,
                             radius_m=float(math.hypot(*xy) / 1000.))
    uncertainty = FLOOR_METHOD_UNCERTAINTY_MM if used == 'floors' else None
    if used == 'walls' and extras.get('inferred_wall_lines'):
        uncertainty = WALL_LINE_UNCERTAINTY_MM if extras.get('inferred_wall_lines') == 1 else 20.
        problems.append(f'slot positions are uncertain by up to about {uncertainty:.0f} mm '
                        f'({extras["inferred_wall_lines"]} wall line(s) hidden in depth were placed using the tray length)')
    if uncertain := (used == 'floors'):
        problems.append(f'slot positions are uncertain by up to about {FLOOR_METHOD_UNCERTAINTY_MM:.0f} mm '
                        '(floor method: walls were not visible in depth)')
    return dict(base, found=True, slots=slots, problems=problems, method=used, position_uncertainty_mm=uncertainty,
                **numbering, rim_height_mm=float(rim_height_mm), **extras)


def drop_z_mm(floor_height_mm, held_height_mm, contact_offset_mm, release_gap_mm=RELEASE_GAP_MM):
    """컵이 놓는 높이(mm, 책상 기준): 칸 바닥 + 실제로 든 물체 높이 + 접촉 보정 + 여유(가정값)."""
    if floor_height_mm is None:
        raise ValueError('floor height of the slot is not measured')
    return float(floor_height_mm) + float(held_height_mm) + float(contact_offset_mm) + float(release_gap_mm)


def slots_config(detection, catalog, *, release_gap_mm=RELEASE_GAP_MM):
    """검출 결과 → load_slots/require_slot이 읽는 설정. 놓는 높이는 칸에 배정된 물체의 높이로 계산한다."""
    if not detection.get('found') or any(s['floor_height_mm'] is None for s in detection['slots'].values()):
        return dict(status='unmeasured', slots={}, rim_height_mm=detection.get('rim_height_mm', RIM_HEIGHT_MM))
    offset = float(catalog['cup_contact_offset_mm'])
    slots = {}
    for number, s in detection['slots'].items():
        object_h = float(catalog['objects'][SLOT_LABEL[number]]['height_mm'])      # 기본 규칙의 물체
        slots[str(number)] = dict(x_mm=s['x_mm'], y_mm=s['y_mm'], floor_height_mm=s['floor_height_mm'],
                                  drop_z_mm=drop_z_mm(s['floor_height_mm'], object_h, offset, release_gap_mm))
    return dict(status='measured', slots=slots, rim_height_mm=float(detection['rim_height_mm']))
