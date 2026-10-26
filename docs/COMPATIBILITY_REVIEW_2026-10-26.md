# 실행 경로 호환성 점검 — 2026-10-26

대상: `/home/song/4dof_ik_pi (Copy)`.
실제 Pi 실행 폴더: `/home/user/4dof_ik_pi_test`.

## 확인한 결함과 수정

| 경계 | 발견한 문제 | 수정 |
| --- | --- | --- |
| 장소 이름 → 지도 라벨 | `시작 위치`와 `시작위치`가 다르게 취급됨 | 원문 정확 일치, 공백 제거 후 정확 일치, 부분 일치 순으로 검색. 여러 후보면 거부 |
| API·메뉴 → HOME 결과 | `home_return_complete`만 보고 종료 코드·최종 고정·게인 복원 실패를 무시하는 경로가 있음 | `home_return_succeeded()`로 성공 판정을 통일. 오류 원문도 함께 전달 |
| HOME → stdout 요약 | 최종 고정·녹화 정리 실패가 JSON 요약에서 누락됨 | 관련 오류 필드를 요약에 포함 |
| 자동 HOME → JSON 파서 | 마지막 JSON이 숫자면 예외, 배열이면 정상 결과를 놓침 | HOME 결과 객체만 선택 |
| unknown 집기 → 트레이 재촬영 | 저장 프레임 제거 요청을 무시해 동일 프레임으로 재시도 | `use_tray_frame`을 실제 명령 생성까지 전달 |
| 연속 집기 → HOME 복구 | 코드 4를 최종 오류처럼 표시. HOME 실패 원인 누락. 정지 직후 집기 재시작 가능 | 복구 상태·재시도 단계 표시, HOME 오류 유지, 정지 후 재시도 차단 |
| HOME 마지막 단계 → 드라이버 | 고정 확인 후 새 드라이버 인수 실패로 확인된 상태를 잃음 | 최종 HOME에서는 기존 드라이버 유지. 중간 단계는 인수 성공 후 교체 |
| Pi 설치 → 시리얼 드라이버 | 설치 목록은 `pyserial>=3.5`, 드라이버는 정확히 `3.5` 요구 | 설치 목록을 `pyserial==3.5`로 맞춤 |

HOME 공통 성공 조건은 종료 코드 0, `home_return_complete=true`,
`final_hold_confirmed=true`, `restoration_verified=true`, 관련 오류 없음이다.
성공 테스트의 HOME 응답도 실제 실행 결과와 같은 필드를 갖도록 수정했다.
모터 속도·게인·좌표·허용 오차·가드 모드는 변경하지 않았다.

## 검증

- Python: `/home/song/miniconda3/envs/lerobot_312/bin/python`.
- 프로젝트 `pytest -q tests`: **275 passed, 13 subtests passed**, 107.03초.
  로그: `/tmp/pi-compat-tests-default.log`.
- 이후 추가한 장소명 수정: `test_place_name_spacing.py`와 `test_base_drive.py`
  **34 passed**, 23.53초. 위 전체 검사와 일부 중복되는 검사다.
- 엄격 모드 `VACUUM_STRICT_GUARDS=1`: 실행 계약·연속 복구·HOME 인계·복귀 영역·
  들어올림·운반 범위·ROS 연동 계약 관련 **37 passed**.
- 이번 경계 결함의 재현 테스트는 수정 전 **8 실패 / 1 통과**,
  공백 이름 테스트는 수정 전 **6 실패 / 4 통과**를 확인했다.
- 변경 Python 파일 문법 검사, `git diff --check` 통과.
- 저장소 전체의 ROS 카메라 노드 직접 수집 검사는 로컬에 `rclpy`가 없어 실행하지 못함.

## Pi 반영과 읽기 전용 확인

- 서버의 `/status`에서 유휴 상태를 확인하고 변경 전 파일 해시를 대조한 뒤 반영했다.
- 기존 Pi의 별도 주석은 보존했다. 설정·보정 파일은 변경하지 않았다.
- 백업: `/home/user/4dof_ik_pi_test/output/compatibility_fix_20261026T133125`.
- 반영 파일: `scripts/base_drive.py`, `scripts/arm_api.py`, `scripts/sort_menu.py`,
  `scripts/return_home.py`, `src/sorting/place_followup.py`, `src/sorting/sort_runner.py`,
  `pi/requirements-pi.txt`.
- API 재시작 완료. `/health`는 `ok=true`, `/status`는 `idle`.
- 현재 Pi의 pyserial 버전: **3.5**. 패키지 재설치는 하지 않았다.
- 실제 지도 라벨을 읽어 `시작 위치`, `시작위치` 모두 저장명 `시작위치`,
  ID **38bada70**으로 해석되는 것을 확인했다. 주행 명령은 보내지 않았다.

## 검증 한계

이 검사는 소프트웨어 경계와 현재 배포 상태를 확인한 것이다.
새 수정으로 팔 이동·흡착·정리·주행을 재실행하지 않았다.
연속 경로의 `hover has no direct-branch solution`은 자세에 따라 여전히 발생할 수 있으며,
기존의 경로 검사를 유지하고 HOME 복구 후 같은 물체를 다시 시도한다.
실제 집기 성공과 물리적인 HOME 도착은 별도의 실행 관찰이 필요하다.
