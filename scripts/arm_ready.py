"""서보 응답과 팔 상태를 확인하고, HOME에 놓인 채 토크만 꺼진 팔은 토크를 켜서 붙잡는다(pi/start_all.sh가 부른다).

  python scripts/arm_ready.py           # 확인만 (읽기 전용)
  python scripts/arm_ready.py --hold    # HOME(또는 접힌 HOME 근처)에서 토크가 꺼져 있으면 켠다 -- 서보에 쓴다

토크를 켜는 것은 그 자리 위치를 붙잡는 것뿐이라 팔이 옮겨 가지 않는다(메뉴에서 Enter로 하던 servo_driver.py hold와 같다).
HOME이 아닌 곳에 있거나 일부만 켜진 팔은 건드리지 않는다. 끝 코드: 0 = 움직일 수 있음, 1 = 사람이 봐야 함.
"""
import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import sort_menu as menu  # noqa: E402

MESSAGES = {
    'ok': '토크 켜짐 - 움직일 수 있습니다.',
    'off_at_home': 'HOME에 놓여 있고 토크가 꺼져 있습니다.',
    'off_near_fold': '접힌 HOME 근처에 있고 토크가 꺼져 있습니다(작업 전에 HOME에 맞춥니다).',
    'off_elsewhere': '토크가 꺼져 있고 HOME이 아닌 곳에 있습니다. 팔을 손으로 받쳐 HOME 자세로 옮긴 뒤 다시 실행하세요.',
    'mixed': '일부 관절만 토크가 켜져 있습니다. 팔을 받치고 servo_driver.py disarm --supported로 모두 끈 뒤 HOME에서 다시 시작하세요.',
    'unknown': '서보가 응답하지 않습니다. 서보 전원(어댑터 전원)이 켜져 있는지, 서보 선이 빠지지 않았는지 확인하세요.',
}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--hold', action='store_true', help='HOME에서 토크가 꺼져 있으면 켠다')
    args = parser.parse_args()
    state = menu.classify_arm(menu.read_arm_state())
    print(f'팔 상태: {state} - {MESSAGES.get(state, state)}')
    if args.hold and state in ('off_at_home', 'off_near_fold'):
        subprocess.run([sys.executable, str(ROOT / 'scripts' / 'servo_driver.py'), 'hold', '--arm', '--supported', '--duration', '1.5'],
                       cwd=ROOT, capture_output=True, text=True, timeout=60)
        state = menu.classify_arm(menu.read_arm_state())
        print('토크를 켜서 팔을 붙잡았습니다.' if state == 'ok' else f'토크를 켜지 못했습니다({state}). {MESSAGES.get(state, "")}')
    return 0 if state in ('ok', 'off_at_home', 'off_near_fold') else 1


if __name__ == '__main__':
    sys.exit(main())
