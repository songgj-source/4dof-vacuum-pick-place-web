"""속도(펄스/초)에 맞는 틱당 펄스 수. 틱(= 펄스수/속도)이 서보 통신 한 번(실측 13~14ms)보다 짧으면 틱이 밀려 속도가 안 나고 떨린다
(2026-10-27: 400펄스/초에 4펄스 = 10ms 틱). 그래서 틱이 약 13.4ms가 되도록 펄스 수를 속도에 맞춰 늘린다.
150→2, 300→4(기존 값 그대로), 400→5, 500→7, 600→8."""
import math

MIN_TICK_S = .0134
MIN_PULSES = 2
MAX_PULSES = 8


def tick_pulses(rate):
    if isinstance(rate, bool) or not isinstance(rate, (int, float)) or not math.isfinite(rate) or rate <= 0:
        raise ValueError('rate must be a positive finite number')
    return max(MIN_PULSES, min(MAX_PULSES, round(rate * MIN_TICK_S)))


# 틱당 펄스로 허용하는 값. 5~8은 400~500펄스/초에서 틱을 통신 한계(약 13.4ms) 이상으로 두려고 쓴다(2026-10-28). 접촉·상승 가까이는 여전히 2 이하만 허용된다(hover_trial).
ALLOWED_STEPS = (1, 2, 4, 5, 6, 7, 8)
