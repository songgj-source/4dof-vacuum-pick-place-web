"""저장해 둔 트레이 프레임의 위치가 지금 화면과 같은지 확인한다. 하드웨어 접근 없음.

트레이 바깥 모서리 3곳(왼쪽 위, 오른쪽 위, 왼쪽 아래)의 작은 조각을 지금 화면에서 찾아(템플릿 매칭) 저장 프레임과의 픽셀 이동량을 잰다.
칸 안에 물체가 들어 있어도 모서리는 영향받지 않는다. 가장 큰 이동량이 허용치(기본 3px) 이하일 때만 '같은 자리'로 본다.
"""
import cv2
import numpy as np

# 저장 프레임(tray_full2)에서 트레이 바깥 모서리 근처의 중심 좌표와 조각 반지름(px)
CORNERS = ((290, 142), (392, 142), (290, 430))
HALF = 22
SEARCH = 24
MIN_SCORE = .55
# 칸 안 물체에 모서리 한 곳이 가려질 수 있다(사용자 요청 2026-10-15: 물체를 칸에 넣은 채로 다음 물체를 옮긴다).
# 일치도가 OCCLUDED_BELOW 미만이면 그 모서리는 가려진 것으로 보고, 나머지 두 곳이 모두 GOOD_SCORE 이상으로 허용치 안에 맞아야 통과한다.
# 모서리 2곳이 맞으면 트레이의 이동과 회전이 모두 확인된다. 가려지지 않은 모서리에 오차가 있으면 그대로 거부한다.
GOOD_SCORE = .9
OCCLUDED_BELOW = .85


TRAY_LENGTH_MM, TRAY_WIDTH_MM = 285., 120.
CORNER_INSET_PX = 12


def tray_corners(stored, depth=None, root=None):
    """비교할 모서리 조각 중심 3곳(영상 픽셀). 저장 프레임(트레이가 비어 있어야 한다)의 깊이로 칸 바닥 3개를 찾고, 알려진 트레이 크기
    (길이 285, 바깥 폭 120mm)로 바깥 모서리 4곳을 로봇 좌표에서 영상으로 옮긴 뒤 안쪽으로 조금 들인 곳이다. 질감이 큰 3곳을 쓴다.
    트레이가 어디에 어떤 각도로 놓여도 그 모서리를 본다(2026-10-19: 예전엔 한 위치의 좌표를 하드코딩해서 트레이를 옮기면 빈 책상을 비교했다).
    깊이·root가 없거나 칸 바닥을 못 찾으면 예전 고정 좌표를 쓴다. 색으로 트레이를 찾는 방식은 조명에 따라 끊겨서 쓰지 않는다."""
    if depth is None or root is None or not isinstance(stored, np.ndarray) or stored.shape != (480, 640, 3):
        return CORNERS
    try:
        from tray.tray_detect import detect_tray, _color_pixel, _locator
        found = detect_tray(stored, depth, root, first_end='far_from_base', method='floors')
        if not found.get('found'):
            return CORNERS
        locator = _locator(root)
        slots = [np.array([found['slots'][k]['x_mm'], found['slots'][k]['y_mm']]) for k in (1, 2, 3)]
        axis = slots[0] - slots[2]
        axis /= np.linalg.norm(axis)
        cross = np.array([-axis[1], axis[0]])
        center = slots[1]
        px = []
        for sa, sb in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            xy = center + sa * TRAY_LENGTH_MM / 2 * axis + sb * TRAY_WIDTH_MM / 2 * cross
            p = _color_pixel(locator, xy / 1000., 55.)
            if p is None:
                return CORNERS
            px.append(np.array(p, float))
    except (ValueError, KeyError, TypeError, OSError):
        return CORNERS
    middle = np.mean(px, axis=0)
    gray = cv2.GaussianBlur(cv2.cvtColor(stored, cv2.COLOR_BGR2GRAY), (3, 3), 0)
    candidates = []
    for point in px:
        inward = middle - point
        q = point + inward / max(np.linalg.norm(inward), 1e-9) * CORNER_INSET_PX
        cx, cy = int(round(q[0])), int(round(q[1]))
        if HALF <= cx <= 640 - HALF and HALF <= cy <= 480 - HALF:
            candidates.append((float(gray[cy - HALF:cy + HALF, cx - HALF:cx + HALF].std()), (cx, cy)))
    if len(candidates) < 3:
        return CORNERS
    candidates.sort(key=lambda c: -c[0])
    return tuple(c[1] for c in candidates[:3])


def _subpixel(result, loc):
    """일치도 지도의 봉우리를 가로·세로 포물선 보간으로 부분 픽셀까지 구한다(가장자리 봉우리는 그대로)."""
    x, y = loc
    fx, fy = float(x), float(y)
    h, w = result.shape
    if 0 < x < w - 1:
        a, b, c = float(result[y, x - 1]), float(result[y, x]), float(result[y, x + 1])
        denominator = a - 2 * b + c
        if denominator < -1e-9:
            fx += max(-.5, min(.5, .5 * (a - c) / denominator))
    if 0 < y < h - 1:
        a, b, c = float(result[y - 1, x]), float(result[y, x]), float(result[y + 1, x])
        denominator = a - 2 * b + c
        if denominator < -1e-9:
            fy += max(-.5, min(.5, .5 * (a - c) / denominator))
    return fx, fy


def tray_offset_px(stored, live, tolerance_px=3., depth=None, root=None):
    for img in (stored, live):
        if not isinstance(img, np.ndarray) or img.shape != (480, 640, 3):
            raise ValueError('480x640 BGR images required')
    g0, g1 = (cv2.GaussianBlur(cv2.cvtColor(i, cv2.COLOR_BGR2GRAY), (3, 3), 0) for i in (stored, live))
    offsets, scores = [], []
    corners = tray_corners(stored, depth, root)
    offsets, scores, vectors = [], [], []
    for cx, cy in corners:
        patch = g0[cy - HALF:cy + HALF, cx - HALF:cx + HALF]
        x0, y0 = max(0, cx - HALF - SEARCH), max(0, cy - HALF - SEARCH)
        area = g1[y0:min(480, cy + HALF + SEARCH), x0:min(640, cx + HALF + SEARCH)]
        if patch.std() < 3 or area.shape[0] < patch.shape[0] or area.shape[1] < patch.shape[1]:
            offsets.append(float('inf'))
            scores.append(0.)
            vectors.append((0., 0.))
            continue
        result = cv2.matchTemplate(area, patch, cv2.TM_CCOEFF_NORMED)
        _, score, _, loc = cv2.minMaxLoc(result)
        fx, fy = _subpixel(result, loc)
        offsets.append(float(np.hypot(x0 + fx + HALF - cx, y0 + fy + HALF - cy)))
        scores.append(float(score))
        vectors.append((float(x0 + fx + HALF - cx), float(y0 + fy + HALF - cy)))
    worst = max(offsets)
    ok = bool(worst <= tolerance_px and min(scores) >= MIN_SCORE)
    occluded = [i for i, sc in enumerate(scores) if sc < OCCLUDED_BELOW]
    if not ok:
        # 평면 위 트레이의 이동·회전은 서로 떨어진 두 점이 정확히 맞으면 모두 확정된다. 한 모서리가 물체·케이블에 가려지면 그 조각은
        # 엉뚱한 곳에서 꽤 높은 일치도(0.87~0.96)로 맞기도 하므로, 일치도로 가려짐을 판단하지 않고 '정확히 맞는 모서리가 2곳 이상'인지 본다.
        good = [i for i in range(len(corners)) if offsets[i] <= tolerance_px and scores[i] >= GOOD_SCORE]
        if len(good) >= 2 and len(good) >= len(corners) - 1:
            ok = True
            occluded = [i for i in range(len(corners)) if i not in good]
            worst = max(offsets[i] for i in good)
    return dict(ok=ok, max_offset_px=worst, offsets_px=offsets, scores=scores, occluded_corners=occluded, corners=[tuple(c) for c in corners], vectors=vectors)



REGISTER_MAX_SHIFT_PX = 40.
REGISTER_MAX_ROTATION_DEG = 6.


def register_tray(stored, live, depth=None, root=None, tolerance_px=3.):
    """트레이가 조금 밀렸거나 돌았을 때 그 이동(회전+평행이동)을 측정한다: 저장 프레임 → 지금 프레임의 영상 평면 강체 변환.
    정확히 맞는(일치도 GOOD_SCORE 이상) 모서리 2곳이면 정확히, 3곳이면 최소제곱으로 푼다. 한 모서리가 물체·케이블에 가려져도 된다.
    이동이 REGISTER_MAX_SHIFT_PX, 회전이 REGISTER_MAX_ROTATION_DEG를 넘거나 정확히 맞는 모서리가 2곳 미만이면 ok=False."""
    check = tray_offset_px(stored, live, tolerance_px=1e9, depth=depth, root=root)
    corners, vectors, scores = check['corners'], check['vectors'], check['scores']
    good = [i for i in range(len(corners)) if scores[i] >= GOOD_SCORE and np.isfinite(check['offsets_px'][i])]
    base = dict(ok=False, matrix=None, good_corners=good, scores=scores)
    if len(good) < 2:
        return dict(base, reason='fewer than two corners match the stored frame')
    src = np.array([corners[i] for i in good], float)
    dst = src + np.array([vectors[i] for i in good], float)
    matrix, _ = cv2.estimateAffinePartial2D(src, dst, method=cv2.LMEDS) if len(good) >= 3 else (None, None)
    if matrix is None:                                   # 2점: 정확한 닮음변환(배율 1로 고정한 강체 변환)
        a, b = src[0], src[1]
        c, d = dst[0], dst[1]
        v0, v1 = b - a, d - c
        theta = np.arctan2(v1[1], v1[0]) - np.arctan2(v0[1], v0[0])
        cos, sin = np.cos(theta), np.sin(theta)
        rot = np.array([[cos, -sin], [sin, cos]])
        matrix = np.hstack([rot, (((c + d) / 2) - rot @ ((a + b) / 2)).reshape(2, 1)])
    else:                                                # 3점: 배율 성분을 버리고 회전+이동만 쓴다
        theta = np.arctan2(matrix[1, 0], matrix[0, 0])
        cos, sin = np.cos(theta), np.sin(theta)
        rot = np.array([[cos, -sin], [sin, cos]])
        matrix = np.hstack([rot, (dst.mean(axis=0) - rot @ src.mean(axis=0)).reshape(2, 1)])
    rotation = float(np.degrees(np.arctan2(matrix[1, 0], matrix[0, 0])))
    center = np.array(corners, float).mean(axis=0)
    shift = float(np.linalg.norm(matrix[:, :2] @ center + matrix[:, 2] - center))
    residual = float(np.max(np.linalg.norm(src @ matrix[:, :2].T + matrix[:, 2] - dst, axis=1)))
    ok = bool(abs(rotation) <= REGISTER_MAX_ROTATION_DEG and shift <= REGISTER_MAX_SHIFT_PX and residual <= 3.)
    return dict(base, ok=ok, matrix=matrix.tolist(), rotation_deg=rotation, shift_px=shift, residual_px=residual,
                reason=None if ok else f'tray moved too far or unevenly (rotation {rotation:.1f} deg, shift {shift:.0f} px, residual {residual:.1f} px)')


EDGE_PAD_X_PX, EDGE_PAD_Y_PX = 62, 75      # 칸 중심들을 감싸는 상자에서 트레이 테두리까지의 여유
EDGE_SEARCH_PX = 40
EDGE_MIN_SCORE = .33
EDGE_MIN_RATIO = 1.15                       # 가장 잘 맞는 자리가 두 번째로 잘 맞는 자리보다 이만큼 뚜렷해야 한다


def edge_registration(stored, live, centers_px):
    """칸이 물체로 꽉 차 바닥·모서리를 못 맞출 때를 위한 보조: 저장 사진에서 트레이 둘레(벽 윤곽)만 오려 지금 사진의 윤곽에서 찾는다.
    칸 안 물체는 윤곽 안쪽을 바꿀 뿐 트레이 테두리는 그대로라 평행이동은 잡힌다(2026-11-02: 세 칸이 다 차서 모서리 맞춤이 실패했다).
    돌아갔는지는 알 수 없어 평행이동만 구한다. 결과: dict(ok, matrix(2x3), shift_px, score, ratio). 뚜렷하게 맞지 않으면 ok=False."""
    centers = np.array(centers_px, float)
    x0, y0 = (centers.min(axis=0) - (EDGE_PAD_X_PX, EDGE_PAD_Y_PX)).astype(int)
    x1, y1 = (centers.max(axis=0) + (EDGE_PAD_X_PX, EDGE_PAD_Y_PX)).astype(int)
    height, width = live.shape[:2]
    if x0 < 0 or y0 < 0 or x1 > width or y1 > height:
        return dict(ok=False, reason='tray region outside the image')

    def edges(image):
        gray = cv2.GaussianBlur(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        return cv2.dilate(cv2.Canny(gray, 40, 120), np.ones((3, 3), np.uint8))

    template = edges(stored)[y0:y1, x0:x1]
    xa, ya = max(0, x0 - EDGE_SEARCH_PX), max(0, y0 - EDGE_SEARCH_PX)
    xb, yb = min(width, x1 + EDGE_SEARCH_PX), min(height, y1 + EDGE_SEARCH_PX)
    result = cv2.matchTemplate(edges(live)[ya:yb, xa:xb], template, cv2.TM_CCORR_NORMED)
    _, best, _, loc = cv2.minMaxLoc(result)
    others = result.copy()
    cv2.circle(others, loc, 12, 0, -1)
    _, second, _, _ = cv2.minMaxLoc(others)
    dx, dy = float(xa + loc[0] - x0), float(ya + loc[1] - y0)
    ratio = float(best / second) if second > 0 else float('inf')
    ok = bool(best >= EDGE_MIN_SCORE and ratio >= EDGE_MIN_RATIO)
    return dict(ok=ok, matrix=[[1., 0., dx], [0., 1., dy]], shift_px=float(np.hypot(dx, dy)), score=float(best), ratio=ratio,
                reason=None if ok else f'tray outline not clear (score {best:.2f}, ratio {ratio:.2f})')


def move_slots(slots, registration, root):
    """저장 프레임에서 찾은 칸 중심(로봇 xy)을 측정한 이동만큼 옮긴다. 칸 바닥 높이는 그대로."""
    from tray.tray_detect import _color_pixel, _locator
    from perception.object_targets import _project
    from perception.box_target import TABLE_PLANE_Z
    locator = _locator(root)
    matrix = np.array(registration['matrix'], float)
    moved = {}
    for number, slot in slots.items():
        floor = slot['floor_height_mm'] if slot['floor_height_mm'] is not None else 12.
        pixel = np.array(_color_pixel(locator, np.array([slot['x_mm'], slot['y_mm']]) / 1000., floor), float)
        new = matrix[:, :2] @ pixel + matrix[:, 2]
        plane_z = TABLE_PLANE_Z - (40. - floor) / 1000.
        xy = (_project(locator, (float(new[0]), float(new[1])), plane_z) + locator.offset_xy) * 1000.
        moved[number] = dict(slot, x_mm=float(xy[0]), y_mm=float(xy[1]), radius_m=float(np.hypot(*xy) / 1000.))
    return moved
