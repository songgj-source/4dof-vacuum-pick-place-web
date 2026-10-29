"""테스트는 가드 배율 1(예전 기준)로 돈다. 배율이 적용되는지는 test_guard_scale.py가 따로 확인한다."""
import os
import sys
from pathlib import Path

os.environ.setdefault('VACUUM_GUARD_SCALE', '1')
os.environ.setdefault('ARM_BATCH_PLAN', '0')        # 한 번에 계획하기: 기존 시험은 물체별 계획 명령 순서를 가정한다. 시험하는 곳에서 켠다
os.environ.setdefault('ARM_PICK_RETRY', '0')        # 집기 실패 재시도: 기존 시험은 첫 실패를 그대로 본다. 시험하는 곳에서 켠다
os.environ.setdefault('ARM_TIDY_VERIFY', '0')       # 정리 뒤 확인 사진도 시험에서 켠 곳만
os.environ.setdefault('ARM_ARRIVAL_PHOTO', '0')     # 그냥 이동한 뒤 찍는 사진: 진짜 카메라를 열지 않게 기본은 끄고, 시험하는 곳에서 켠다

# 이 conda 환경에는 autolab_perception의 'perception' 패키지가 깔려 있어 우리 src/perception을 가린다. 시험 파일을 따로 돌려도 src가 먼저 오게 한다.
_ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(_ROOT / 'src'), str(_ROOT / 'scripts')]
