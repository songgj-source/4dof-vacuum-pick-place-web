"""손목 카메라를 열어 둔 채 집힘을 확인한다(동작이 느려지지 않게). 하드웨어 접근 없음."""
import threading
import time

import numpy as np

from motion import hold_check
from motion.wrist_stream import WristStream


class FakeCam:
    def __init__(self, frames):
        self.frames, self.index, self.released = frames, 0, False

    def read(self):
        time.sleep(.002)
        frame = self.frames[min(self.index, len(self.frames) - 1)]
        self.index += 1
        return True, frame

    def release(self):
        self.released = True


def _frame(value):
    return np.full((480, 640, 3), value, np.uint8)


def test_stream_skips_warmup_returns_fresh_frames_and_releases_camera():
    cam = FakeCam([_frame(i) for i in range(1, 60)])
    stream = WristStream('dev', opener=lambda d: cam).start()
    first, stamp = stream.latest(timeout=1.)
    assert first is not None and first[0, 0, 0] > 5                      # 처음 5장(노출 안 맞음)은 버린다
    later, later_stamp = stream.latest(after=stamp, timeout=1.)
    assert later_stamp > stamp and later[0, 0, 0] >= first[0, 0, 0]
    stream.stop()
    assert cam.released and not stream.thread.is_alive()


def test_stream_reports_open_failure_without_raising():
    def broken(device):
        raise OSError('no camera')
    stream = WristStream('dev', opener=broken).start()
    assert stream.latest(timeout=.1) == (None, None)
    stream.stop()
    assert 'no camera' in stream.error


def test_stream_modes_abort_only_when_clearly_not_held():
    assert hold_check.hold_decision(False, 'stream') == 'abort'
    assert hold_check.hold_decision(False, 'stream-record') == 'continue'
    assert hold_check.hold_decision(None, 'stream') == 'continue'
    assert hold_check.hold_decision(True, 'stream') == 'continue'
    assert set(hold_check.STREAM_MODES) == {'stream', 'stream-record'}


def test_pick_auto_uses_stream_for_hold_check_without_disabling_continuous_lift():
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'scripts/pick_auto.py').read_text().replace(' ', '')
    assert "args.hold_check in ('off',*STREAM_MODES)".replace(' ', '') in src      # 연속 들어올리기(lift_to_carry)가 stream에서도 켜진다
    assert "args.hold_check in ('enforce','record')".replace(' ', '') in src      # 예전 방식만 사진마다 카메라를 다시 연다
    assert "default=os.environ.get('ARM_HOLD_CHECK')or'stream'" in src
    assert 'lift_tilted' in src and 'wrist_stream[0].stop()' in src


def test_hold_check_cannot_judge_a_white_object_on_the_white_table():
    # 2026-10-28 Pi: 흰 상자는 접촉 사진에서 물체로 안 잡혀 '물체 없음 = 안 붙음'으로 오판해 잘 집은 것을 실패로 멈췄다
    white = np.full((480, 640, 3), 235, np.uint8)
    verdict = hold_check.check_hold(white, white)
    assert verdict['held'] is None and 'cannot verify' in verdict['reason']
    assert hold_check.hold_decision(verdict['held'], 'stream') == 'continue'


def test_hold_check_still_fails_a_visible_object_left_on_the_table():
    contact = np.full((480, 640, 3), 235, np.uint8)
    contact[150:400, 230:500] = (30, 30, 200)                   # 빨간 물체가 컵 밑에 크게 보인다
    lifted = np.full((480, 640, 3), 235, np.uint8)               # 들어 올렸더니 화면에서 사라졌다
    verdict = hold_check.check_hold(contact, lifted)
    assert verdict['held'] is False and 'left on the table' in verdict['reason']
    held = hold_check.check_hold(contact, contact.copy())
    assert held['held'] is True


WHITE_DATA = __import__('pathlib').Path(__file__).resolve().parent / 'data/hold_white_20261028'
WHITE_TRIALS = ['20261028T083718', '20261028T083809', '20261028T083901']


def _white(trial):
    import cv2
    return (cv2.imread(str(WHITE_DATA / f'{trial}_before_suction.png')), cv2.imread(str(WHITE_DATA / f'{trial}_lifted_check.png')))


def test_white_box_that_was_really_held_passes_on_real_photos():
    # 2026-10-28 Pi: 이 세 장은 잘 집었는데 '안 붙음'으로 멈췄다
    for trial in WHITE_TRIALS:
        contact, lifted = _white(trial)
        verdict = hold_check.check_hold(contact, lifted)
        assert verdict['held'] is True, (trial, verdict['reason'])
        assert verdict['metrics']['structure_match'] >= .8


def test_white_box_left_on_the_table_fails():
    import cv2
    contact, _ = _white('20261028T083901')
    # 카메라가 물러나 책상 위 상자가 작아진 모의 사진(컵·팔 부품은 카메라에 붙어 그대로)
    matrix = np.float32([[.8, 0, .2 * 320], [0, .8, .2 * 240]])
    left = cv2.warpAffine(contact, matrix, (640, 480), borderMode=cv2.BORDER_REPLICATE)
    left[240:, 270:460] = contact[240:, 270:460]
    left[440:] = contact[440:]
    assert hold_check.check_hold(contact, left)['held'] is False
    gone = np.full((480, 640, 3), 200, np.uint8)                          # 상자가 사라진 빈 책상
    assert hold_check.check_hold(contact, gone)['held'] is False


COLOR_DATA = __import__('pathlib').Path(__file__).resolve().parent / 'data/hold_color_20261029'


def _color(trial):
    import cv2
    return (cv2.imread(str(COLOR_DATA / f'{trial}_before_suction.png')), cv2.imread(str(COLOR_DATA / f'{trial}_lifted_check.png')))


def test_tilted_or_shifted_held_objects_are_not_failed_on_real_photos():
    # 2026-10-29 Pi: 정육면체(면적 1.25배)·줄자(0.85배)를 잘 집었는데 '안 붙음'으로 흡착을 끄고 실패 처리했다
    for trial in ('20261029T013202', '20261029T001703'):
        contact, lifted = _color(trial)
        verdict = hold_check.check_hold(contact, lifted)
        assert verdict['held'] is True, (trial, verdict['reason'])


def test_colored_object_left_on_the_table_still_fails():
    import cv2
    for trial in ('20261029T013202', '20261029T001703'):
        contact, _ = _color(trial)
        matrix = np.float32([[.65, 0, .35 * 320], [0, .65, .35 * 240]])        # 카메라가 물러나 물체가 0.65배로 작아진 모의 사진
        left = cv2.warpAffine(contact, matrix, (640, 480), borderMode=cv2.BORDER_CONSTANT, borderValue=(235, 235, 235))      # 줄어든 만큼 책상이 보인다
        left[240:, 270:460] = contact[240:, 270:460]                            # 컵·볼트는 카메라에 붙어 그대로
        left[440:] = contact[440:]
        assert hold_check.check_hold(contact, left)['held'] is False, trial
