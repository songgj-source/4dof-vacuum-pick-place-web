#!/usr/bin/env bash
# 노트북에서 Pi의 로봇팔 서버를 조작한다(ssh로 pi/arm_api_ctl.sh 실행).
#   bash pi/remote.sh start | stop | restart | status | log | attach | check
#   ROBOT=user@192.168.0.31 REMOTE_DIR=4dof_ik_pi_test 로 바꿀 수 있다(deploy_to_robot.sh와 같은 기본값).
set -euo pipefail
ROBOT="${ROBOT:-user@192.168.0.31}"
REMOTE_DIR="${REMOTE_DIR:-4dof_ik_pi_test}"
SSH=(ssh -t); [ -n "${ARM_SSH_KEY:-}" ] && SSH+=(-i "$ARM_SSH_KEY")
case "${1:-}" in
  log|attach) exec "${SSH[@]}" "$ROBOT" "bash ~/$REMOTE_DIR/pi/arm_api_ctl.sh $*" ;;
  start|stop|restart|status|check) exec "${SSH[@]}" "$ROBOT" "FORCE=${FORCE:-0} bash ~/$REMOTE_DIR/pi/arm_api_ctl.sh $*" ;;
  *) sed -n '2,4p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
