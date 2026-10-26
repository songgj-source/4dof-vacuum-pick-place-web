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


def _name(blob, masks, chromatic, area):
    """이름 판정. 흰색은 덩어리 전체의 밝은 무채색 비율, 나머지는 색이 남은 픽셀의 우세 색."""
    fractions = {name: float(mask[blob].mean()) for name, mask in masks.items()}
    count = int(chromatic[blob].sum())
    shares = {name: (float(masks[name][blob].sum()) / count if count else 0.)
              for name in ('red_cube', 'tape_measure')}
    if fractions['white_box'] >= WHITE_MIN_FRACTION:
        return 'white_box', fractions, count, shares
    if count >= max(MIN_CHROMATIC_PX, MIN_CHROMATIC_SHARE * area):
        best = max(shares, key=shares.get)
        if shares[best] >= CHROMATIC_DOMINANCE:
            return best, fractions, count, shares
    return 'unknown', fractions, count, shares


def classify_objects(bgr, depth, intrinsics=None):
    _validate(bgr, depth)
    height = table_height_mm(depth, intrinsics)
    raised = ((height > MIN_HEIGHT_MM) & (depth > 0)).astype(np.uint8)
    raised = cv2.morphologyEx(raised, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    raised = cv2.morphologyEx(raised, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    count, labels, stats, centers = cv2.connectedComponentsWithStats(raised)
    masks, chromatic = _color_masks(bgr)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    registered = object_registry.load()
    found = []
    for i in range(1, count):
        x, y, w, h, area = (int(s) for s in stats[i])
        if area < MIN_AREA_PX or y < IGNORE_TOP_PX or x > IGNORE_RIGHT_X:
            continue
        blob = labels == i
        label, fractions, count, shares = _name(blob, masks, chromatic, area)
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
        found.append(dict(label=label, name=display_name(label), bbox_px=[x, y, w, h], center_px=[float(c) for c in centers[i]],
                          top_center_px=[float(xs.mean()), float(ys.mean())],
                          area_px=area, height_mm=float(np.median(height[blob])), top_height_mm=top_mm,
                          color_fractions=fractions, chromatic_pixels=count, chromatic_shares=shares, signature=sig,
                          usable_for_motion=False, physical_validated=False))
    return found
