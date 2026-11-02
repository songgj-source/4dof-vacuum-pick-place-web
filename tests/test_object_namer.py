"""등록 안 된 새 물체에 비전 모델이 붙인 이름 — 안내 문구에만 쓰고, 실패해도 기존처럼 동작한다."""
import sys
from pathlib import Path

import numpy as np

sys.path[:0] = [str(Path(__file__).resolve().parents[1] / 'scripts'), str(Path(__file__).resolve().parents[1] / 'src')]
import arm_llm
from perception import object_namer


def _frame():
    return np.random.default_rng(1).integers(0, 255, (480, 640, 3), dtype=np.uint8)


def setup_function(_):
    object_namer._cache.clear()


def test_clean_keeps_only_the_name():
    assert object_namer.clean('박스 (추정)') == '박스'
    assert object_namer.clean('"펜".\n설명 줄') == '펜'
    assert object_namer.clean('') is None
    assert object_namer.clean('이것은 책상 위에 놓인 아주 긴 문장입니다') is None


def test_name_is_cached_for_the_same_crop():
    calls = []
    client = lambda images, prompt, timeout: calls.append(1) or '박스 (추정)'
    frame = _frame()
    assert object_namer.name_object(frame, [200, 200, 50, 50], client) == '박스'
    assert object_namer.name_object(frame, [200, 200, 50, 50], client) == '박스'
    assert len(calls) == 1


def test_failure_gives_none_and_is_retried():
    def down(images, prompt, timeout):
        raise OSError('ollama down')
    frame = _frame()
    assert object_namer.name_object(frame, [200, 200, 50, 50], down) is None
    assert object_namer.name_object(frame, [200, 200, 50, 50], lambda i, p, t: '펜') == '펜'


def test_switch_off(monkeypatch):
    monkeypatch.setenv('ARM_NAMER', '0')
    assert object_namer.name_object(_frame(), [200, 200, 50, 50], lambda i, p, t: '박스') is None


def test_color_names_from_measured_values():
    name = object_namer.color_name
    assert name(17., 133., 61.) == '갈색'           # 2026-10-28 화면의 갈색 상자
    assert name(17., 133., 81.) == '갈색'           # 같은 상자, 밝은 조명
    assert name(18., 215., 135.) == '노란'          # 줄자
    assert name(120., 200., 150.) == '파란'
    assert name(60., 150., 150.) == '초록색'
    assert name(2., 200., 180.) == '빨간'
    assert name(170., 80., 200.) == '분홍색'
    assert name(None, 10., 220.) == '흰' and name(None, 10., 120.) == '회색' and name(None, 10., 40.) == '검은'
    assert name(None, None, None) is None


def test_label_joins_color_and_name_without_repeating():
    assert object_namer.label('갈색', '박스') == '갈색 박스'
    assert object_namer.label('갈색', '갈색 박스') == '갈색 박스'
    assert object_namer.label('노란', '노랑 펜') == '노랑 펜' and object_namer.label('파란', '파우치') == '파란 파우치'
    assert object_namer.label('갈색', None) == '갈색 물체'
    assert object_namer.label(None, '박스') == '박스'
    assert object_namer.label(None, None) is None


def test_guess_suffix_shows_color_and_name():
    objects = lambda *pairs: dict(objects=[dict(label='unknown', color=c, guess=g) for c, g in pairs])
    assert arm_llm.unknown_guess(objects(('갈색', '박스'))) == '(갈색 박스 같음)'
    assert arm_llm.unknown_guess(objects(('갈색', '박스'), ('노란', '펜'), ('갈색', '박스'))) == '(갈색 박스, 노란 펜 같음)'
    assert arm_llm.unknown_guess(objects(('갈색', None))) == '(갈색 물체 같음)'
    assert arm_llm.unknown_guess(objects((None, None))) == ''


def test_classify_reports_measured_color_for_unknown_blob():
    import cv2
    from perception import object_classify
    bgr = np.full((480, 640, 3), (200, 200, 200), np.uint8)
    depth = np.full((480, 640), 600, np.uint16)
    bgr[250:310, 250:310] = cv2.cvtColor(np.full((60, 60, 3), (17, 133, 61), np.uint8), cv2.COLOR_HSV2BGR)
    depth[250:310, 250:310] = 570
    item = object_classify.classify_objects(bgr, depth)[0]
    assert item['label'] == 'unknown' and object_namer.color_name(*item['color_hsv']) == '갈색'


def test_json_answer_is_parsed_and_generic_or_unsure_names_are_dropped():
    parse = object_namer.parse_answer
    assert parse('{"name": "건전지", "confidence": 0.8}') == '건전지'
    assert parse('```json\n{"name": "립밤 (추정)", "confidence": 0.5}\n```') == '립밤'
    assert parse('{"name": "물체", "confidence": 0.9}') is None          # 막연한 이름
    assert parse('{"name": "모름", "confidence": 0.1}') is None
    assert parse('{"name": "펜", "confidence": 0.1}') is None            # 확신이 너무 낮다
    assert parse('{"name": "펜"}') == '펜'                                # 확신 값이 없으면 이름을 믿는다
    assert parse('지우개') == '지우개'                                    # 예전처럼 이름 한 줄만 와도 된다
    assert parse('') is None and parse('{broken') is None


def test_prompt_carries_the_measured_facts_and_both_pictures_are_sent():
    prompt = object_namer.build_prompt(dict(color='갈색', size_mm=(52., 31.), height_mm=22.))
    assert '갈색' in prompt and '5.2×3.1cm' in prompt and '2.2cm' in prompt and 'JSON' in prompt
    assert '측정한 값' not in object_namer.build_prompt(None)
    seen = []
    client = lambda images, prompt, timeout: seen.append((len(images), prompt)) or '{"name": "필통", "confidence": 0.7}'
    assert object_namer.name_object(_frame(), [200, 200, 60, 40], client, hints=dict(color='파란')) == '필통'
    assert seen[0][0] == 2 and '파란' in seen[0][1]                       # 확대 사진 + 위치를 표시한 전체 사진


def test_hints_use_the_depth_to_estimate_the_size():
    item = dict(color_hsv=[17., 133., 61.], top_height_mm=22., bbox_px=[100, 100, 50, 30], center_px=[125., 115.])
    depth = np.full((480, 640), 600, np.uint16)
    hints = object_namer.hints_for(item, depth)
    assert hints['color'] == '갈색' and hints['height_mm'] == 22.
    assert abs(hints['size_mm'][0] - 50 * 600 / 502.67) < .1 and abs(hints['size_mm'][1] - 30 * 600 / 523.44) < .1
    assert 'size_mm' not in object_namer.hints_for(item)                    # 깊이가 없으면 크기는 뺀다


def test_crop_is_enlarged_for_small_objects_and_keeps_context():
    frame = _frame()
    small = object_namer._crop(frame, [300, 200, 30, 30])
    assert min(small.shape[:2]) >= 300                                    # 작은 물체는 키운다
    assert max(object_namer._crop(frame, [10, 10, 400, 400]).shape[:2]) <= 512
    context = object_namer._context(frame, [300, 200, 30, 30])
    assert context.shape[1] <= 640 and (context != frame).any()
