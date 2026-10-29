"""서보 게인(P·D·I)이 임시값으로 남았을 때 기준값으로 되돌린다(어깨·팔꿈치·베이스).

홈 복귀 같은 동작은 안정화를 위해 어깨·팔꿈치 게인을 잠시 바꾸고 끝나면 되돌린다. 그 프로세스가 도중에 끊기면(서버 재시작·전원·강제 종료)
임시값이 서보에 남아 다음 복귀가 'gain baseline changed'로 막힌다. 기본은 읽기만 한다. --execute를 주면 기준값(P16·D32·I0)으로 쓴다.
팔은 움직이지 않는다(게인 레지스터만 쓴다). 팔이 받쳐져 있고 멈춘 상태에서 실행한다.
  python scripts/restore_gains.py            # 지금 값만 보여 준다
  python scripts/restore_gains.py --execute  # 기준값이 아니면 되돌리고 읽어서 확인
"""
import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--execute', action='store_true', help='기준값이 아닌 게인을 기준값으로 쓴다(없으면 읽기만)')
    args = parser.parse_args(argv)
    from hardware.servo_mapping import MappingProfile
    from hardware.servo_transport import ServoTransport
    from motion.trial_gains import BASELINE, restore_baseline_gains
    profile = MappingProfile.load(ROOT / 'config/servo_driver_candidate.json', ROOT)
    out = ROOT / 'output/gain_restore' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')
    out.mkdir(parents=True, exist_ok=True)
    with ServoTransport(profile) as bus:
        report = restore_baseline_gains(bus, out, execute=args.execute)
    names = {'1': 'base', '2': 'shoulder', '3': 'elbow'}
    for motor, row in report.items():
        cur = row['current']
        state = '기준값' if row['baseline_ok'] else f"기준과 다름 (기준 P{BASELINE['p']} D{BASELINE['d']} I{BASELINE['i']})"
        line = f"{names.get(motor, motor):9s} P{cur['p']} D{cur['d']} I{cur['i']} startup{cur['startup']} lock{cur['lock']}  → {state}"
        if row['changed']:
            after = row['after']
            line += f"  ⇒ 쓴 뒤 P{after['p']} D{after['d']} I{after['i']} {'확인됨' if row['verified'] else '확인 실패'}"
        print(line)
    ok = all(row['verified'] for row in report.values())
    (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    if not args.execute and not all(row['baseline_ok'] for row in report.values()):
        print('기준과 다른 게인이 있습니다. 되돌리려면 --execute로 다시 실행하세요.')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
