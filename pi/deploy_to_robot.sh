#!/usr/bin/env bash
# 노트북에서 팀 로봇 Pi로 우리 팔 코드를 올리고 설치한다(팀원의 파일·환경은 건드리지 않는다).
#
#   bash pi/deploy_to_robot.sh                       # 올리기 + 설치(--no-service)
#   bash pi/deploy_to_robot.sh --dry-run             # 접속·복사 없이 할 일만 출력
#   ROBOT=user@192.168.0.31 ARM_SSH_KEY=~/.ssh/id_ed25519_workstation6 bash pi/deploy_to_robot.sh   (주소·키 지정)
#   설치 옵션을 Pi의 install.sh에 넘기려면 -- 뒤에: bash pi/deploy_to_robot.sh -- --no-apt --openni2 /opt/openni2-orbbec
#
# 복사하는 것: git이 관리하는(또는 새로 만든) 코드·설정·보정 파일. 안 하는 것: output/ 실행 기록, 이 노트북의 .venv, x86 OpenNI2,
# ros2_slam/(팀 주행 코드 사본 - Pi에서는 팀 스크립트가 ~/ros2_slam_robot으로 따로 배포·빌드한다).
# 올린 뒤 Pi에서: bash ~/4dof_ik_pi_test/pi/arm_api_ctl.sh start   (또는 노트북에서 pi/remote.sh start)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOT="${ROBOT:-user@192.168.0.31}"
REMOTE_DIR="${REMOTE_DIR:-4dof_ik_pi_test}"
DRY=0
INSTALL_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY=1; shift ;;
    --) shift; INSTALL_ARGS=("$@"); break ;;
    -h|--help) sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "알 수 없는 옵션: $1"; exit 2 ;;
  esac
done
[ ${#INSTALL_ARGS[@]} -gt 0 ] || INSTALL_ARGS=(--no-service)

SSH_CTL="$(mktemp -u /tmp/arm-deploy-XXXXXX)"
SSH_OPTS=(-o ControlMaster=auto -o "ControlPath=$SSH_CTL" -o ControlPersist=60 -o ConnectTimeout=6)
[ -n "${ARM_SSH_KEY:-}" ] && SSH_OPTS+=(-i "$ARM_SSH_KEY")      # 예: ARM_SSH_KEY=~/.ssh/id_ed25519_workstation6
cleanup() { ssh "${SSH_OPTS[@]}" -O exit "$ROBOT" 2>/dev/null || true; }
trap cleanup EXIT

cd "$ROOT"
# 보낼 파일 목록: 추적 중인 파일 + 아직 추적 안 하지만 무시 대상이 아닌 새 파일. 지역 값이 든 것과 큰 것은 뺀다.
list_files() {
  git ls-files -co --exclude-standard -z |
    grep -zvE '^(output/|portfolio/|하드웨어 설명/|tmp/|\.venv/|config/devices\.json$|pi/arm-api\.env$|calibration/openni2_redist/|ros2_slam/)'
}
COUNT="$(list_files | tr -cd '\0' | wc -c)"
SIZE="$(list_files | xargs -0 du -ck 2>/dev/null | tail -1 | cut -f1)"
echo "보낼 파일: ${COUNT}개, 약 $((SIZE / 1024)) MB  →  $ROBOT:~/$REMOTE_DIR"
echo "  ※ output/ik_resume_20261012 등 실행에 필요한 보정 기록은 git 추적 파일이라 아래에서 따로 포함합니다."

# output 아래에서 실행에 꼭 필요한 보정 기록만 포함(실행 기록과 이 노트북의 트레이 사진은 제외)
need_output() { git ls-files -z -- 'output/ik_resume_20261012' 2>/dev/null; }
if [ "$DRY" = 1 ]; then
  echo "[실행 안 함] ssh $ROBOT 'mkdir -p ~/$REMOTE_DIR'"
  echo "[실행 안 함] rsync -az --files-from=<목록> ./ $ROBOT:~/$REMOTE_DIR/"
  echo "[실행 안 함] ssh -t $ROBOT 'cd ~/$REMOTE_DIR && bash pi/install.sh ${INSTALL_ARGS[*]}'"
  echo "포함되는 output 보정 기록: $(need_output | tr -cd '\0' | wc -c)개"
  exit 0
fi

echo "== Pi에 접속(비밀번호를 한 번 묻습니다)"
if ! ssh "${SSH_OPTS[@]}" "$ROBOT" 'echo "접속 성공: $(hostname), $(uname -m)"'; then
  echo "접속하지 못했습니다. Pi가 켜져 있고 이 노트북과 같은 네트워크인지(ping ${ROBOT#*@}), 사용자 이름이 맞는지 확인하세요. 다른 주소면 ROBOT=사용자@주소 로 지정합니다."
  exit 1
fi
ssh "${SSH_OPTS[@]}" "$ROBOT" "mkdir -p ~/$REMOTE_DIR"
echo "== 코드 복사"
{ list_files; need_output; } | rsync -az --from0 --files-from=- -e "ssh ${SSH_OPTS[*]}" ./ "$ROBOT:~/$REMOTE_DIR/"
echo "== Pi에서 설치(${INSTALL_ARGS[*]})"
ssh -t "${SSH_OPTS[@]}" "$ROBOT" "cd ~/$REMOTE_DIR && bash pi/install.sh ${INSTALL_ARGS[*]}"
cat <<EOF

올렸습니다. 다음:
  노트북에서:  bash pi/remote.sh check      # 장치·드라이버 점검
               bash pi/remote.sh start      # 서버 켜기 (팀의 카메라 프로그램이 돌고 있으면 거부)
  브라우저:    http://${ROBOT#*@}:8000     (토큰은 install.sh가 보여 준 값)
EOF
