"""들어 올린 직후 손목 사진으로 물체가 컵에 붙어 올라왔는지 판단한다. 하드웨어 접근 없음.

원리: 접촉 직전 사진과 20mm 든 사진을 비교한다. 물체가 컵에 붙어 있으면 컵과 같이 올라와서 화면에서 같은 자리·같은 크기로 남고,
붙지 않았으면 책상에 남아 카메라가 물러나므로 작아지고(축소) 자리가 옮겨진다. 물체는 책상(밝은 흰색)과 다른 덩어리(색이 있거나 어두운
부분)로 찾고, 컵 근처의 가장 가까운 큰 덩어리를 쓴다. 접촉 사진에 물체가 없으면 '집을 물체가 없음'으로 본다.

범위: 한 조명·책상에서 확인한 잠정 기준이다. 집히지 않은 실제 사진은 아직 없다(축소 모델로만 시험). 판정 불가(None)는 막지 않는다.
"""
import cv2
import numpy as np

SEED = (360, 250)              # 접촉 순간 컵 위로 물체 윗면이 보이는 자리(손목 영상 기준)
MIN_AREA_PX = 3000
MIN_PRESENT_FRACTION = .06     # 물체가 화면에서 차지해야 하는 최소 비율
IOU_MIN = .75                  # 기록용(metrics)만 쓴다. 판정에는 안 쓴다
AREA_RATIO_RANGE = (.88, 1.15)  # 예전 기준(기록용)
SHRINK_FAIL_BELOW = .70        # 이보다 작아지면 책상에 남은 것
GROW_UNSURE_ABOVE = 1.6
BOTTOM_IGNORE_Y = 440          # 화면 아래 검은 판(팔 부품)


def _valid(img):
    return isinstance(img, np.ndarray) and img.shape == (480, 640, 3) and img.dtype == np.uint8


def _blob(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = (((hsv[..., 1] > 90) & (hsv[..., 2] > 40)) | (hsv[..., 2] < 70)).astype(np.uint8)
    mask[BOTTOM_IGNORE_Y:] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    best, nearest = 0, 1e18
    for i in range(1, count):
        if stats[i][4] < MIN_AREA_PX:
            continue
        ys, xs = np.nonzero(labels == i)
        distance = float(np.min((xs - SEED[0]) ** 2 + (ys - SEED[1]) ** 2))
        if distance < nearest:
            best, nearest = i, distance
    return (labels == best) if best else np.zeros(mask.shape, bool)


# 색 대비가 없는 물체(흰 책상 위 흰 상자)용: 윤곽·무늬(밝기 기울기)가 접촉 사진과 들어올린 사진에서 같은 자리에 그대로 남는지 본다.
# 붙어 올라온 물체는 카메라와 함께 움직여 윤곽·스티커가 그대로이고, 책상에 남은 물체는 카메라가 물러나 작아지고 옮겨진다.
# 2026-10-28 Pi 흰 상자 3장: 잘 집었을 때 일치도 0.83~0.90, 같은 사진을 10% 줄인 모의 책상 남음 0.49 이하, 물체가 사라지면 0.02.
STRUCTURE_TRUE_MIN = .70
STRUCTURE_FALSE_MAX = .55
STRUCTURE_MIN_TEXTURE = 3.        # 비교할 영역의 평균 윤곽 세기가 이보다 약하면(밋밋한 흰 면) 비교할 것이 없다
STRUCTURE_SHIFT_PX = 14
CUP_MASK = (slice(240, None), slice(270, 460))     # 컵·볼트: 카메라에 붙어 있어 어느 쪽이든 똑같이 보인다(비교에서 뺀다)
STRUCTURE_WINDOW = (slice(40, 230), slice(60, 600))


def _edges(img):
    gray = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32), (5, 5), 0)
    edges = cv2.magnitude(cv2.Sobel(gray, cv2.CV_32F, 1, 0), cv2.Sobel(gray, cv2.CV_32F, 0, 1))
    edges[CUP_MASK] = 0
    edges[BOTTOM_IGNORE_Y:] = 0
    return edges


def structure_match(contact_bgr, lifted_bgr):
    """윤곽 일치도(−1~1)와 접촉 사진 영역의 평균 윤곽 세기. 조금 어긋나도 되게 ±STRUCTURE_SHIFT_PX 안에서 가장 잘 맞는 값."""
    before, after = _edges(contact_bgr), _edges(lifted_bgr)
    template = before[STRUCTURE_WINDOW]
    texture = float(template.mean())
    if texture < 1e-6 or float(template.std()) < 1e-6:
        return 0., texture
    height, width = template.shape
    y0, x0 = STRUCTURE_WINDOW[0].start, STRUCTURE_WINDOW[1].start
    best = -1.
    for dy in range(-STRUCTURE_SHIFT_PX, STRUCTURE_SHIFT_PX + 1, 2):
        for dx in range(-STRUCTURE_SHIFT_PX, STRUCTURE_SHIFT_PX + 1, 2):
            window = after[y0 + dy:y0 + dy + height, x0 + dx:x0 + dx + width]
            if window.shape != template.shape or float(window.std()) < 1e-6:
                continue
            best = max(best, float(np.corrcoef(template.ravel(), window.ravel())[0, 1]))
    return best, texture


def check_hold(contact_bgr, lifted_bgr):
    base = dict(usable_for_motion=False, physical_validated=False, scope='wrist photos, one lighting/table, provisional thresholds')
    if not (_valid(contact_bgr) and _valid(lifted_bgr)):
        return dict(base, held=None, reason='unreadable photos; cannot verify', metrics={})
    before, after = _blob(contact_bgr), _blob(lifted_bgr)
    area_before, area_after = int(before.sum()), int(after.sum())
    metrics = dict(area_contact=area_before, area_lifted=area_after)
    if area_before < MIN_PRESENT_FRACTION * before.size:
        # 접촉 사진에서 물체를 못 찾았다. 물체를 색·어두움으로 찾기 때문에 흰 책상 위 흰 상자는 안 잡힌다(2026-10-28 Pi: 잘 집었는데 '안 붙음'으로 멈췄다).
        # 색으로는 못 찾았다. '안 붙었다'는 증거가 아니므로 윤곽 일치도로 따로 판단하고, 그것도 판단할 수 없으면 막지 않는다.
        score, texture = structure_match(contact_bgr, lifted_bgr)
        metrics.update(structure_match=score, structure_texture=texture)
        if texture < STRUCTURE_MIN_TEXTURE:
            return dict(base, held=None, reason='object not visible at contact (low contrast, e.g. plain white); cannot verify', metrics=metrics)
        if score >= STRUCTURE_TRUE_MIN:
            return dict(base, held=True, reason=f'outline kept its place in the lifted photo (match {score:.2f}; low-contrast object)', metrics=metrics)
        if score <= STRUCTURE_FALSE_MAX:
            return dict(base, held=False, reason=f'outline changed between contact and lift (match {score:.2f} <= {STRUCTURE_FALSE_MAX}; low-contrast object left on the table)', metrics=metrics)
        return dict(base, held=None, reason=f'outline match {score:.2f} is between the pass and fail limits; cannot verify', metrics=metrics)
    if area_after == 0:
        return dict(base, held=False, reason='object not in the lifted photo (left on the table)', metrics=metrics)
    union = int((before | after).sum())
    iou = float((before & after).sum()) / union
    ratio = area_after / area_before
    metrics.update(iou=iou, area_ratio=ratio)
    # 붙어 올라온 물체도 컵 위에서 기울거나 밀려 모양이 바뀐다. 2026-10-29 Pi에서 잘 집은 정육면체(1.25배)·줄자(0.85배)를 '안 붙음'으로 멈췄다
    # (옆 골판지 벽이 물체와 한 덩어리로 잡혀 면적도 어긋났다). 그래서 면적·겹침이 조금 달라진 것은 증거로 보지 않는다.
    # 책상에 남은 물체는 카메라가 물러나 분명히 작아진다(0.7배 밑). 그때만 '안 붙음'이다. 너무 커진 것(1.6배 넘음)은 판단 불가.
    if ratio < SHRINK_FAIL_BELOW:
        return dict(base, held=False, metrics=metrics, reason=f'object shrank to {ratio:.2f}x between contact and lift (a held object keeps its size; it was left on the table)')
    if ratio > GROW_UNSURE_ABOVE:
        return dict(base, held=None, metrics=metrics, reason=f'object grew to {ratio:.2f}x between contact and lift; cannot verify')
    return dict(base, held=True, metrics=metrics, reason=f'object still large in the lifted photo (size {ratio:.2f}x, overlap {iou:.2f}); a tilted or shifted held object is accepted')


# stream / stream-record: 카메라를 열어 둔 채 받은 사진으로 비교한다(wrist_stream). 동작이 느려지지 않아 연속 들어올리기와 같이 쓸 수 있다.
# stream은 enforce처럼 안 붙었으면 멈추고, stream-record는 기록만 한다.
MODES = ('enforce', 'record', 'off', 'stream', 'stream-record')
STREAM_MODES = ('stream', 'stream-record')


def hold_decision(held, mode):
    """'abort'는 명확히 '안 붙었다'(False)이고 enforce(stream)일 때만. 판정 불가(None)는 기존 동작대로 계속한다."""
    if mode not in MODES:
        raise ValueError(f'hold check mode must be one of {MODES}')
    return 'abort' if (held is False and mode in ('enforce', 'stream')) else 'continue'
