"""테스트는 가드 배율 1(예전 기준)로 돈다. 배율이 적용되는지는 test_guard_scale.py가 따로 확인한다."""
import os

os.environ.setdefault('VACUUM_GUARD_SCALE', '1')
