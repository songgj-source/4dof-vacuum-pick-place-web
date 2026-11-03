"""탑뷰(Astra) RGB+깊이에서 책상 위 물체를 찾고 색으로 이름을 붙인다.

하드웨어 접근 없음. 책상 평면을 깊이로 적합해 평면 위로 솟은 덩어리를 찾고,
덩어리의 색 비율로 빨간 정육면체/줄자/흰 상자를 구분한다. 색이 어느 쪽에도 충분히
맞지 않으면 추측하지 않고 'unknown'으로 둔다.
등록 물체(object_registry, custom_N)는 덩어리 특징(색·면적·윗면 높이)이 등록 값과 맞으면 그 이름이 우선한다
(등록할 때 기본 물체와 헷갈리지 않는지 검사한다). 덩어리마다 특징을 signature로 함께 돌려준다.

범위: 한 조명·책상 조건에서 맞춘 후보 분류다. 집기 목표나 물리 검증된 값이 아니다.
알려진 한계: '흰색'은 채도가 낮고 밝은 덩어리이므로 밝은 무채색 물체와 구분하지 못한다.
깊이와 색 영상은 DEPTH_MINUS_TOP_CENTER_PX만큼 어긋나 있다. 색은 이만큼 옮겨 깊이 픽셀에 맞춘 뒤 읽는다(2026-11-03).
center_px·bbox_px는 깊이 좌표다. 색 영상 위치는 이 값을 빼서 구한다.
"""
import hashlib

import cv2
import numpy as np

from perception import object_registry

DISPLAY_NAME = {'red_cube': 'red cube', 'tape_measure': 'tape measure', 'white_box': 'white box', 'unknown': 'unknown'}
MIN_HEIGHT_MM = 8.
MIN_AREA_PX = 600
# 화면 위쪽 멀티탭은 책상 위 물체가 아니다.
# 예전에는 트레이가 늘 오른쪽이라 x 440 오른쪽을 통째로 버렸다(IGNORE_RIGHT_X). 트레이를 가운데에 두자 오른쪽 물체 셋이 모두 빠졌다(2026-11-02 사진).
# 트레이는 큰 덩어리·칸 안 판정(arm_api)으로 따로 걸러지므로 위치로 버리지 않는다.
IGNORE_TOP_PX = 60
# 깊이 픽셀 - 이 값 = 같은 점의 색 픽셀(object_targets·tray_detect와 같은 값). 덩어리는 깊이로 찾으므로 색을 이만큼 옮겨 맞춘 뒤 읽는다.
# 맞추지 않으면 지름 45px 정육면체에서 색의 1/3 이상을 옆 책상에서 읽었다(2026-11-03: 흰색 비율 0.23 → 맞춘 뒤 0.08).
DEPTH_MINUS_TOP_CENTER_PX = (17., 5.5)
# 어두워서 색 정보가 사라진 픽셀(V 낮음)은 판정에서 뺀다. 색이 남은 픽셀만으로 이름을 정한다.
CHROMA_MIN_SAT = 60
CHROMA_MIN_VAL = 28   # 35→28(2026-11-02 Pi): 그림자 속 빨간 정육면체의 밝기(V)가 중앙값 31이라 색 픽셀이 46개(<60)뿐이어서 새 물체로 판정돼 건너뛰었다
# 줄자는 진한 노란색이다. 색상(10~30)만 보면 갈색 골판지·상자도 들어와 새 물체를 줄자로 불렀다. 밝기로는 못 가른다(밝은 조명의
# 갈색 상자 밝기 중앙값 96, 줄자 118~136). 채도가 확실히 다르다: 색상 범위 안 픽셀 중 채도 170 이상 비율이 줄자 0.64~0.93,
# 갈색 상자 0.02~0.04(2026-10-28 실제 프레임 3장). 채도 150·밝기 85 하한은 조명이 바뀌면 진짜 줄자도 떨어뜨렸다(채도 중앙 114인 순간).
TAPE_VIVID_SAT = 170
TAPE_VIVID_SHARE = .30
MIN_CHROMATIC_PX = 60
MIN_CHROMATIC_SHARE = .06   # 덩어리 면적 대비
CHROMATIC_DOMINANCE = .6    # 색이 남은 픽셀 중 한 색이 차지해야 하는 비율
WHITE_MIN_FRACTION = .5
PLANE_INLIER_MM = 4.
# 세 물체 중 가장 큰 것(7cm)도 화면에서 약 90px이다. 이보다 큰 덩어리(트레이 등)는 색이 맞아도 이름을 붙이지 않는다.
MAX_OBJECT_PX = 150
# 가장 낮은 물체(흰 상자 24mm)보다 한참 낮은 덩어리(트레이 칸 바닥 약 10mm)는 색이 비슷해도 이름을 붙이지 않는다.
MIN_NAMED_HEIGHT_MM = 16.
# 깊이 카메라 내부 파라미터 가정값: 프로젝트의 깊이 모델(conditional_fx_fy_original_D)과 같은 값.
DEFAULT_INTRINSICS = dict(fx=502.67051745358543, fy=523.437961343891, cx=325.98544285719885,
                          cy=242.88435652180573,
                          distortion=(0.25138274669452126, -0.7219120565944923, -0.05587307479203079,
                                      0.0020875971400429725, 0.7786024477249887))


def display_name(label):
    return DISPLAY_NAME.get(label) or object_registry.names().get(label, label)


def _validate(bgr, depth):
    if not isinstance(bgr, np.ndarray) or bgr.shape != (480, 640, 3) or bgr.dtype != np.uint8:
        raise ValueError('RGB must be 480x640x3 uint8 BGR')
    if not isinstance(depth, np.ndarray) or depth.shape != (480, 640):
        raise ValueError('depth must be 480x640')


def _camera_points_mm(depth, intrinsics):
    """유효한 깊이 픽셀의 3D 점(mm, 카메라 광축 좌표). 렌즈 왜곡을 보정한다."""
    valid = depth > 0
    if int(valid.sum()) < 1000:
        raise ValueError('not enough valid depth')
    v, u = np.nonzero(valid)
    k = np.array([[intrinsics['fx'], 0, intrinsics['cx']], [0, intrinsics['fy'], intrinsics['cy']], [0, 0, 1.]])
    pixels = np.stack([u, v], 1).astype(np.float64).reshape(-1, 1, 2)
    normalized = cv2.undistortPoints(pixels, k, np.asarray(intrinsics['distortion'], float)).reshape(-1, 2)
    z = depth[valid].astype(np.float64)
    return valid, np.c_[normalized * z[:, None], z]


def _fit_plane(points):
    """책상 평면: 3D 점에서 결정적인 RANSAC 후 인라이어 최소제곱. 법선은 카메라 쪽을 향한다."""
    rng = np.random.default_rng(0)
    sample = points[rng.choice(len(points), min(len(points), 20000), replace=False)]
    best = None
    for _ in range(300):
        a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        count = int((np.abs((sample - a) @ normal) < PLANE_INLIER_MM).sum())
        if best is None or count > best[0]:
            best = (count, a, normal)
    if best is None or best[0] < .3 * len(sample):
        raise ValueError('table plane not found')
    _, origin, normal = best
    inliers = points[np.abs((points - origin) @ normal) < PLANE_INLIER_MM]
    center = inliers.mean(axis=0)
    # 3x3 공분산의 최소 고유벡터 = 평면 법선 (점 수와 무관하게 작은 계산)
    normal = np.linalg.eigh(np.cov((inliers - center).T))[1][:, 0]
    if normal @ (-center) < 0:        # 카메라(원점) 쪽을 향하게
        normal = -normal
    return center, normal


def table_height_mm(depth, intrinsics=None):
    """각 픽셀이 책상 평면 위로 솟은 수직 높이(mm). 깊이가 없는 픽셀은 0.

    카메라 시선을 따라 잰 깊이 차이가 아니라 평면까지의 수직 거리라서, 비스듬히 볼 때
    높이가 부풀지 않는다.
    """
    intrinsics = intrinsics or DEFAULT_INTRINSICS
    depth = np.asarray(depth)
    # 계획 한 번에 같은 깊이 영상으로 5번 불린다(물체·트레이·칸 검사, 0.1초씩). 평면 맞춤은 시드가 고정돼 결과가 같으므로
    # 내용이 같은 영상이면 저장해 둔 결과의 복사본을 돌려준다(호출한 쪽이 고쳐 써도 저장본은 그대로).
    key = (hashlib.sha1(np.ascontiguousarray(depth).view(np.uint8)).hexdigest(), depth.shape, depth.dtype.str,
           tuple(sorted(intrinsics.items())))
    if key in _HEIGHT_CACHE:
        return _HEIGHT_CACHE[key].copy()
    valid, points = _camera_points_mm(depth, intrinsics)
    center, normal = _fit_plane(points)
    height = np.zeros(depth.shape, np.float64)
    height[valid] = (points - center) @ normal
    if len(_HEIGHT_CACHE) >= 4:
        _HEIGHT_CACHE.pop(next(iter(_HEIGHT_CACHE)))
    _HEIGHT_CACHE[key] = height
    return height.copy()


_HEIGHT_CACHE = {}


def align_color_to_depth(bgr):
    """색 영상을 깊이 좌표로 옮긴다: 결과[v, u] = 색[v - dy, u - dx]. 가장자리는 이웃 픽셀로 채운다."""
    dx, dy = DEPTH_MINUS_TOP_CENTER_PX
    return cv2.warpAffine(bgr, np.float32([[1, 0, dx], [0, 1, dy]]), (bgr.shape[1], bgr.shape[0]),
                          flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_REPLICATE)


def _color_masks(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    hue, sat, val = (hsv[..., k].astype(int) for k in range(3))
    chromatic = (sat >= CHROMA_MIN_SAT) & (val >= CHROMA_MIN_VAL)
    return {
        'red_cube': chromatic & ((hue <= 5) | (hue >= 160)),
        'tape_measure': chromatic & (hue >= 10) & (hue <= 30),
        'white_box': (sat < 60) & (val > 100),
    }, chromatic


def _vivid_tape(hsv):
    """줄자 색상 범위이면서 채도가 높은(진한 노랑) 픽셀. 갈색은 같은 색상이라도 채도가 낮다."""
    hue, sat = hsv[..., 0].astype(int), hsv[..., 1].astype(int)
    return (hue >= 10) & (hue <= 30) & (sat >= TAPE_VIVID_SAT)


def _name(blob, masks, chromatic, area, vivid=None):
    """이름 판정. 흰색은 덩어리 전체의 밝은 무채색 비율, 나머지는 색이 남은 픽셀의 우세 색.
    줄자는 우세 색이 노랑 계열이고, 그중 진한(채도 높은) 픽셀이 충분할 때만이다(아니면 갈색 새 물체)."""
    fractions = {name: float(mask[blob].mean()) for name, mask in masks.items()}
    count = int(chromatic[blob].sum())
    shares = {name: (float(masks[name][blob].sum()) / count if count else 0.)
              for name in ('red_cube', 'tape_measure')}
    if fractions['white_box'] >= WHITE_MIN_FRACTION:
        return 'white_box', fractions, count, shares
    if count >= max(MIN_CHROMATIC_PX, MIN_CHROMATIC_SHARE * area):
        best = max(shares, key=shares.get)
        if shares[best] >= CHROMATIC_DOMINANCE:
            if best == 'tape_measure' and vivid is not None:
                band = int(masks['tape_measure'][blob].sum())
                if not band or float((vivid & masks['tape_measure'])[blob].sum()) / band < TAPE_VIVID_SHARE:
                    return 'unknown', fractions, count, shares            # 노랑 계열이지만 탁하다: 갈색 상자·골판지
            return best, fractions, count, shares
    return 'unknown', fractions, count, shares


OPEN_KERNEL = np.ones((5, 5), np.uint8)
CLOSE_KERNEL = np.ones((9, 9), np.uint8)


SPLIT_PIECE_MAX_TOP_MM = 50.   # 큰 덩어리를 나눈 조각 중 이보다 높은 것은 트레이 벽·칸 안 물체다(책상 물체는 45mm 이하)


def _blobs(raised, height=None):
    """솟은 픽셀 → 덩어리 마스크 목록. 잡음을 지우고(열기) 깊이 구멍을 메운다(닫기).
    닫기는 몇 픽셀 틈도 메워서, 트레이 벽에 바짝 붙은 물체가 트레이와 한 덩어리가 돼 '큰 덩어리'로 통째로 빠졌다
    (2026-10-28 흰 상자: 깊이에서는 70×70px 별도 덩어리였는데 트레이와 합쳐져 176×174px). 닫은 덩어리가 물체보다 크면
    닫기 전 원래 연결로 다시 나눈다. 원래 이어진 트레이는 한 덩어리로 남고, 붙어 있던 물체만 떨어진다."""
    opened = cv2.morphologyEx(raised, cv2.MORPH_OPEN, OPEN_KERNEL)
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, CLOSE_KERNEL)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(closed)
    raw_labels = None
    blobs = []
    for i in range(1, count):
        mask = labels == i
        if max(int(stats[i][2]), int(stats[i][3])) > MAX_OBJECT_PX:
            if raw_labels is None:
                raw_labels = cv2.connectedComponents(raised)[1]
            parts = [j for j in np.unique(raw_labels[mask]) if j]
            if len(parts) > 1:
                for j in parts:
                    piece = (mask & (raw_labels == j)).astype(np.uint8)
                    piece = cv2.morphologyEx(cv2.morphologyEx(piece, cv2.MORPH_OPEN, OPEN_KERNEL), cv2.MORPH_CLOSE, CLOSE_KERNEL)
                    piece = piece.astype(bool) & mask
                    if not piece.any():
                        continue
                    # 트레이를 못 찾은 사진에서는 이렇게 나눈 트레이 조각(벽 약 53mm 이상)이 '새 물체'로 잡혔다(2026-10-28).
                    # 물체 크기의 벽 높이 조각만 버린다. 트레이 크기 조각은 '큰 덩어리'로 남겨 둔다(아래에서 unknown, 크기로 걸러진다)
                    ys, xs = np.nonzero(piece)
                    small = max(int(xs.max() - xs.min()), int(ys.max() - ys.min())) + 1 <= MAX_OBJECT_PX
                    if small and height is not None and float(np.percentile(height[piece], 90)) > SPLIT_PIECE_MAX_TOP_MM:
                        continue
                    blobs.append(piece)
                continue
        blobs.append(mask)
    return blobs


# 기본 물체의 깊이 윗면 높이(mm): 실제 프레임 20여 장에서 잰 값. 흰 상자 21~31(중앙 23.5), 정육면체 39~41.5, 줄자 33~34.5(실제 30mm에 깊이 +3mm).
BUILTIN_TOP_MM = {'white_box': 24., 'red_cube': 40., 'tape_measure': 33.}
BUILTIN_TOP_TOL_MM = 8.

SPLIT_MIN_BLOB_PX = 3600       # 이보다 작은 덩어리는 한 물체다(한 물체는 3000px 안쪽). 줄자 하나의 흰 글씨·반사가 가짜 흰 상자를 만들었다(2026-10-30: 흰 상자 2개로 계획 거부)
SPLIT_MIN_PART_PX = 350        # 붙은 물체를 색으로 나눌 때 한 물체가 되려면 이 색이 덩어리 안에 이만큼은 있어야 한다
SPLIT_GROW_PX = 3               # 색 조각을 이만큼 부풀려 물체 몸통(어두운 옆면·글씨)까지 포함한다


def _split_touching(blobs, masks, chromatic, vivid, height=None):
    """서로 붙어 한 덩어리가 된 등록 물체들(줄자·흰 상자·정육면체)을 색별로 나눈다(2026-10-29: 세 물체가 붙어 있어 깊이로는 한 덩어리 → 전부 unknown이라
    '흰 상자를 못 찾음'). 한 덩어리 안에 등록 색(빨강·진한 노랑·흰색)이 둘 이상, 각각 SPLIT_MIN_PART_PX 이상이면 색 조각마다 한 물체로 본다.
    트레이 같은 큰 덩어리(MAX_OBJECT_PX 초과)와 색이 하나뿐인 덩어리는 그대로 둔다. 조각은 부풀려 서로 겹치는 픽셀은 버린다.
    SPLIT_MIN_BLOB_PX보다 작은 덩어리는 흰색을 빼고 빨강·진한 노랑만 본다. 흰 글씨는 줄자 한 개 안에도 있지만, 빨강과 진한 노랑이
    함께 있으면 붙은 두 물체다(2026-11-03: 정육면체와 줄자가 붙어 2772px 한 덩어리 → 둘 다 새 물체로 나왔다).
    작은 덩어리는 깊이로 한 번 더 확인한다: 조각마다 MIN_AREA_PX 이상이고 깊이로 잰 윗면 높이가 그 물체 높이(BUILTIN_TOP_MM)와 맞을 때만 나눈다.
    아니면 그대로 둔다. 두 색이 섞인 새 물체 하나를 둘로 쪼개거나, 작은 조각이 최소 크기에 걸려 장애물이 통째로 사라지지 않게 한다."""
    colors = {'red_cube': masks['red_cube'], 'tape_measure': masks['tape_measure'] & vivid, 'white_box': masks['white_box']}
    out = []
    grow = np.ones((2 * SPLIT_GROW_PX + 1, 2 * SPLIT_GROW_PX + 1), np.uint8)
    for blob in blobs:
        ys, xs = np.nonzero(blob)
        if not len(xs) or max(int(xs.max() - xs.min()) + 1, int(ys.max() - ys.min()) + 1) > MAX_OBJECT_PX or len(xs) < 2 * SPLIT_MIN_PART_PX:
            out.append(blob)
            continue
        small = len(xs) < SPLIT_MIN_BLOB_PX
        parts = {}
        for name, mask in colors.items():
            if small and name == 'white_box':
                continue
            inside = (mask & blob).astype(np.uint8)
            inside = cv2.morphologyEx(inside, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            count, labels, stats, _ = cv2.connectedComponentsWithStats(inside)
            pieces = [labels == i for i in range(1, count) if stats[i][4] >= SPLIT_MIN_PART_PX]
            if pieces:
                parts[name] = max(pieces, key=lambda piece: int(piece.sum()))          # 색마다 가장 큰 조각 하나
        if len(parts) < 2:
            out.append(blob)
            continue
        grown = {name: cv2.dilate(piece.astype(np.uint8), grow).astype(bool) & blob for name, piece in parts.items()}
        regions = {}
        for name, region in grown.items():
            others = np.zeros_like(blob, bool)
            for other, other_region in grown.items():
                if other != name:
                    others |= other_region
            regions[name] = region & ~others
        if small and not all(_fits_builtin(name, region, height) for name, region in regions.items()):
            out.append(blob)
            continue
        out.extend(regions.values())
    return out


def _fits_builtin(name, region, height):
    """나눈 조각이 그 기본 물체로 볼 만한가: 최소 크기 이상이고, 깊이가 있으면 윗면 높이(상위 10%)가 그 물체 높이와 맞는다."""
    if int(region.sum()) < MIN_AREA_PX:
        return False
    if height is None:
        return True
    values = height[region]
    values = values[np.isfinite(values)]          # 깊이가 없는 픽셀은 뺀다
    return bool(values.size) and abs(float(np.percentile(values, 90)) - BUILTIN_TOP_MM[name]) <= BUILTIN_TOP_TOL_MM


def classify_objects(bgr, depth, intrinsics=None):
    _validate(bgr, depth)
    height = table_height_mm(depth, intrinsics)
    raised = ((height > MIN_HEIGHT_MM) & (depth > 0)).astype(np.uint8)
    color = align_color_to_depth(bgr)            # 깊이 덩어리와 같은 자리의 색을 읽는다
    masks, chromatic = _color_masks(color)
    hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    vivid = _vivid_tape(hsv)
    registered = object_registry.load()
    found = []
    for blob in _split_touching(_blobs(raised, height), masks, chromatic, vivid, np.where(depth > 0, height, np.nan)):
        ys, xs = np.nonzero(blob)
        x, y = int(xs.min()), int(ys.min())
        w, h, area = int(xs.max()) - x + 1, int(ys.max()) - y + 1, int(blob.sum())
        center = (float(xs.mean()), float(ys.mean()))
        if area < MIN_AREA_PX or y < IGNORE_TOP_PX:
            continue
        label, fractions, count, shares = _name(blob, masks, chromatic, area, vivid)
        blob_median = float(np.median(height[blob & (depth > 0)]))
        top_mm = float(np.percentile(height[blob & (depth > 0)], 90))
        sig = object_registry.signature(hsv, chromatic, blob & (depth > 0), top_mm, blob_median, area)
        mine = object_registry.match(sig, registered) if registered else None
        if mine:
            label = mine
        if max(w, h) > MAX_OBJECT_PX or blob_median < MIN_NAMED_HEIGHT_MM:
            label = 'unknown'
        # 기본 물체는 색만으로 이름을 붙이면 같은 색의 다른 물체(높이가 다른 흰 상자 등)를 그 물체로 착각해 카탈로그 높이로 집는다(2026-11-02).
        # 깊이로 잰 윗면 높이가 그 물체의 높이와 BUILTIN_TOP_TOL_MM 넘게 다르면 새 물체로 본다(새 물체는 잰 높이로 집는다).
        if label in BUILTIN_TOP_MM and abs(top_mm - BUILTIN_TOP_MM[label]) > BUILTIN_TOP_TOL_MM:
            label = 'unknown'
        # 윗면: 덩어리에서 가장 높은 쪽(상위 10% 높이에서 5mm 이내) 픽셀의 중심. 깊이 좌표 기준이다.
        blob_height = np.where(blob & (depth > 0), height, -np.inf)
        top = blob & (blob_height >= top_mm - 5.)
        ys, xs = np.nonzero(top)
        # 윗면 픽셀의 최소 넓이 회전 사각형(깊이 좌표, 둘레 순서). 칸에 놓을 때 물체 방향을 따지는 데 쓴다(slot_wall)
        corners = cv2.boxPoints(cv2.minAreaRect(np.c_[xs, ys].astype(np.float32))) if len(xs) >= 5 else None
        pixels = blob & (depth > 0)
        color_hsv = [sig['hue'], float(np.median(hsv[..., 1][pixels])), float(np.median(hsv[..., 2][pixels]))] if pixels.any() else None
        found.append(dict(label=label, name=display_name(label), bbox_px=[x, y, w, h], center_px=list(center),
                          color_hsv=color_hsv,
                          top_center_px=[float(xs.mean()), float(ys.mean())],
                          top_corners_px=None if corners is None else [[float(a), float(b)] for a, b in corners],
                          area_px=area, height_mm=float(np.median(height[blob])), top_height_mm=top_mm,
                          color_fractions=fractions, chromatic_pixels=count, chromatic_shares=shares, signature=sig,
                          usable_for_motion=False, physical_validated=False))
    return found
