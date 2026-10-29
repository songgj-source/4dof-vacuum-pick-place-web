"""운반·접근·HOME 복귀 속도 상한을 500펄스/초로 올리고, 틱이 통신 한계(약 13.4ms) 밑으로 밀리지 않게 틱당 펄스를 맞춘다(2026-10-28). 하드웨어 접근 없음."""
import pytest

import sort_menu
from motion import home_return_profile, tick_pulses
from sorting import place_followup


def test_pulses_follow_the_rate_and_keep_tick_at_comm_limit():
    assert [tick_pulses.tick_pulses(r) for r in (50, 150, 300, 400, 500, 600)] == [2, 2, 4, 5, 7, 8]      # 150·300은 예전 값 그대로
    for rate in (150, 300, 400, 500, 600):
        assert .0125 <= tick_pulses.tick_pulses(rate) / rate <= .0145
    with pytest.raises(ValueError):
        tick_pulses.tick_pulses(float('nan'))


def test_home_profile_accepts_500_and_uses_matching_ticks():
    profile = home_return_profile.return_profile(600.)
    assert profile['pulses_per_step'] == 8 and abs(profile['tick_s'] - 8 / 600.) < 1e-9
    assert home_return_profile.return_profile(300.)['pulses_per_step'] == 4
    with pytest.raises(ValueError):
        home_return_profile.return_profile(601.)


def test_home_return_command_raises_tracking_limit_with_speed():
    def limit(rate):
        command = place_followup.home_return_command('.', rate, 'out')
        return command[command.index('--tracking-limit') + 1] if '--tracking-limit' in command else None
    assert (limit(150.), limit(300.), limit(600.)) == (None, '32', '64')
    with pytest.raises(ValueError):
        place_followup.home_return_command('.', 601., 'out')


def test_menu_runs_place_and_home_return_at_600_with_8_pulse_ticks():
    common = sort_menu.COMMON
    value = lambda flag: common[common.index(flag) + 1]
    assert (value('--place-rate'), value('--place-pulses-per-step'), value('--home-rate')) == ('600', '8', '600')


def test_slow_moves_are_600_but_contact_stays_150():
    from pathlib import Path
    pick_auto = (Path(__file__).resolve().parents[1] / 'scripts/pick_auto.py').read_text().replace(' ', '')
    assert 'SLOW_RATE_PPS=600.' in pick_auto and 'CONTACT_RATE_PPS=150.' in pick_auto
    assert "CONTACT_RATE_PPSifname.startswith(('contact','probe','recenter','rise'))" in pick_auto
    assert '--descent-rate\',type=float,default=600.' in pick_auto and '--held-rate\',type=float,default=600.' in pick_auto


def test_tray_slot_too_far_is_not_blamed_on_the_object():
    # 2026-10-29: 1번 칸이 41.6cm라 놓을 수 없었는데 '빨간 정육면체가 너무 멉니다'로 나와 물체 문제로 보였다
    text = sort_menu.brief_message('tray slot radius 0.416 m outside the 0.16-0.41 m pick envelope (red_cube)')
    assert '놓을 칸이 로봇에서 너무 멉니다(41.6cm, 최대 41cm)' in text and '트레이를 로봇 쪽으로' in text and '정육면체' not in text
    assert '너무 멉니다(41.6cm, 최대 41cm). 로봇 쪽으로 옮기세요.' in sort_menu.brief_message('red_cube radius 0.416 m outside the 0.16-0.41 m pick envelope')
