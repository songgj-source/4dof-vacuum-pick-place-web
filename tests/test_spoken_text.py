"""웹 화면의 소리 내기 전 문장 다듬기(spoken): 괄호·쌍점·단위를 읽기 좋게 풀어 쓴다. node가 없으면 건너뛴다."""
import json
import shutil
import subprocess
from pathlib import Path

import pytest

HTML = Path(__file__).resolve().parents[1] / 'web/index.html'


def run(fn, *texts):
    source = HTML.read_text()
    code = source[source.index('function spoken(text)'):source.index('function speak(text')]
    out = subprocess.run(['node', '-e', 'globalThis.window = globalThis; globalThis.addEventListener = () => {};\n' + code + f'console.log(JSON.stringify({json.dumps(list(texts), ensure_ascii=False)}.map({fn})))'],
                         capture_output=True, text=True, check=True).stdout
    return json.loads(out)


def spoken(*texts):
    return run('spoken', *texts)


def brief(*texts):
    return run('brief', *texts)


pytestmark = pytest.mark.skipif(shutil.which('node') is None, reason='node 없음')


def test_slot_and_guess_parentheses_are_read_naturally():
    text, = spoken('칸 안: 줄자(3번 칸). 책상 위: 새 물체 1개(박스, 펜 같음).')
    assert '(' not in text and ':' not in text
    assert '줄자 3번 칸' in text and '박스나 펜 같은 것' in text and text.startswith('칸 안에는')


def test_units_and_reach_details_are_spelled_out():
    text, = spoken('줄자는 너무 가깝습니다(12cm, 최소 16cm). 로봇에서 16~30cm 안으로 옮겨 주세요. 각도 오차 -0.2°')
    assert '12cm' not in text and '16에서 30센티미터' in text and '마이너스 0.2도' in text


def test_plain_sentences_are_unchanged():
    plain = '책상 정리를 마쳤습니다.'
    assert spoken(plain) == [plain]


def test_arrival_report_is_reduced_to_what_will_be_done():
    report = ('책상에 도착해 다시 찍었습니다. 책상 위: 빨간 정육면체, 줄자. 칸 안: 흰 상자(2번 칸). '
              '닿지 않아 뺐습니다: 새 물체는 너무 멉니다(32cm, 최대 30cm). 로봇에서 16~30cm 안으로 옮겨 주세요. 빨간 정육면체, 줄자를 정리하겠습니다.')
    assert brief(report) == ['닿지 않는 물체는 뺍니다. 빨간 정육면체, 줄자를 정리하겠습니다.']


def test_situation_keeps_only_the_finding_and_check_keeps_the_result():
    situation = "책상에서 빨간 정육면체, 줄자를 찾았습니다. 책상 정리를 준비했습니다. '실행'이나 '정리해'라고 하면 시작합니다."
    check = '정리 확인: 정리가 덜 됐습니다. 칸 안: 줄자(3번 칸). 책상에 남은 물체: 흰 상자.'
    assert brief(situation, check) == ['책상에서 빨간 정육면체, 줄자를 찾았습니다.', '정리 확인: 정리가 덜 됐습니다.']


def test_empty_desk_report_is_not_silenced():
    assert brief('책상에 도착해 다시 찍었습니다. 책상 위에 물체가 없습니다. 팔이 닿는 곳에 정리할 물체가 없습니다.') == ['책상 위에 물체가 없습니다. 팔이 닿는 곳에 정리할 물체가 없습니다.']
