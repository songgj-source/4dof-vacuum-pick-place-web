"""탑뷰(Astra) RGB+깊이에서 책상 위 물체를 찾고 색으로 이름을 붙인다.

하드웨어 접근 없음. 책상 평면을 깊이로 적합해 평면 위로 솟은 덩어리를 찾고,
덩어리의 색 비율로 빨간 정육면체/줄자/흰 상자를 구분한다. 색이 어느 쪽에도 충분히
맞지 않으면 추측하지 않고 'unknown'으로 둔다.
등록 물체(object_registry, custom_N)는 덩어리 특징(색·면적·윗면 높이)이 등록 값과 맞으면 그 이름이 우선한다
(등록할 때 기본 물체와 헷갈리지 않는지 검사한다). 덩어리마다 특징을 signature로 함께 돌려준다.

범위: 한 조명·책상 조건에서 맞춘 후보 분류다. 집기 목표나 물리 검증된 값이 아니다.
알려진 한계: '흰색'은 채도가 낮고 밝은 덩어리이므로 밝은 무채색 물체와 구분하지 못한다.
center_px는 깊이 덩어리 중심이라 색 영상에서 보이는 위치보다 약 15~20px 오른쪽으로
치우쳐 나온다(깊이-색 정합 오차 후보). 집기 위치로 쓰기 전에 보정이 필요하다.
"""
import hashlib

import cv2
import numpy as np

from perception import object_registry

DISPLAY_NAME = {'red_cube': 'red cube', 'tape_measure': 'tape measure', 'white_box': 'white box', 'unknown': 'unknown'}
MIN_HEIGHT_MM = 8.
MIN_AREA_PX = 600
# 화면 위쪽 멀티탭, 오른쪽 골판지 트레이는 책상 위 물체가 아니다.
IGNORE_TOP_PX = 60
IGNORE_RIGHT_X = 440
# 어두워서 색 정보가 사라진 픽셀(V 낮음)은 판정에서 뺀다. 색이 남은 픽셀만으로 이름을 정한다.
CHROMA_MIN_SAT = 60
CHROMA_MIN_VAL = 35
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


SPLIT_MIN_BLOB_PX = 3600       # 이보다 작은 덩어리는 한 물체다(한 물체는 3000px 안쪽). 줄자 하나의 흰 글씨·반사가 가짜 흰 상자를 만들었다(2026-10-30: 흰 상자 2개로 계획 거부)
SPLIT_MIN_PART_PX = 350        # 붙은 물체를 색으로 나눌 때 한 물체가 되려면 이 색이 덩어리 안에 이만큼은 있어야 한다
SPLIT_GROW_PX = 3               # 색 조각을 이만큼 부풀려 물체 몸통(어두운 옆면·글씨)까지 포함한다


def _split_touching(blobs, masks, chromatic, vivid):
    """서로 붙어 한 덩어리가 된 등록 물체들(줄자·흰 상자·정육면체)을 색별로 나눈다(2026-10-29: 세 물체가 붙어 있어 깊이로는 한 덩어리 → 전부 unknown이라
    '흰 상자를 못 찾음'). 한 덩어리 안에 등록 색(빨강·진한 노랑·흰색)이 둘 이상, 각각 SPLIT_MIN_PART_PX 이상이면 색 조각마다 한 물체로 본다.
    트레이 같은 큰 덩어리(MAX_OBJECT_PX 초과)와 색이 하나뿐인 덩어리는 그대로 둔다. 조각은 부풀려 서로 겹치는 픽셀은 버린다."""
    colors = {'red_cube': masks['red_cube'], 'tape_measure': masks['tape_measure'] & vivid, 'white_box': masks['white_box']}
    out = []
    grow = np.ones((2 * SPLIT_GROW_PX + 1, 2 * SPLIT_GROW_PX + 1), np.uint8)
    for blob in blobs:
        ys, xs = np.nonzero(blob)
        if not len(xs) or max(int(xs.max() - xs.min()) + 1, int(ys.max() - ys.min()) + 1) > MAX_OBJECT_PX or len(xs) < SPLIT_MIN_BLOB_PX:
            out.append(blob)
            continue
        parts = {}
        for name, mask in colors.items():
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
        for name, region in grown.items():
            others = np.zeros_like(blob, bool)
            for other, other_region in grown.items():
                if other != name:
                    others |= other_region
            out.append(region & ~others)
    return out


def classify_objects(bgr, depth, intrinsics=None):
    _validate(bgr, depth)
    height = table_height_mm(depth, intrinsics)
    raised = ((height > MIN_HEIGHT_MM) & (depth > 0)).astype(np.uint8)
    masks, chromatic = _color_masks(bgr)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    vivid = _vivid_tape(hsv)
    registered = object_registry.load()
    found = []
    for blob in _split_touching(_blobs(raised, height), masks, chromatic, vivid):
        ys, xs = np.nonzero(blob)
        x, y = int(xs.min()), int(ys.min())
        w, h, area = int(xs.max()) - x + 1, int(ys.max()) - y + 1, int(blob.sum())
        center = (float(xs.mean()), float(ys.mean()))
        if area < MIN_AREA_PX or y < IGNORE_TOP_PX or x > IGNORE_RIGHT_X:
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
