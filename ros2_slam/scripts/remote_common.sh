# Shared by remote_*.sh (sourced, not executed): ssh plumbing for running
# things on the robot (Raspberry Pi) from the laptop.
#
# Environment:
#   ROBOT          ssh target (default user@192.168.0.31)
#   REMOTE_WS      workspace on the robot (default ~/ros2_slam_robot, see deploy_to_robot.sh)
#   ROS_DOMAIN_ID  used on the robot (default 42, must match the laptop)

ROBOT="${ROBOT:-user@192.168.0.31}"
REMOTE_WS="${REMOTE_WS:-ros2_slam_robot}"
DOMAIN="${ROS_DOMAIN_ID:-42}"

# Prefix for commands run in a robot tmux session: tmux sessions get the tmux
# server's environment, not ours, so set the ROS env explicitly.
REMOTE_ROS_ENV="export ROS_DISTRO=jazzy ROS_DOMAIN_ID=${DOMAIN} RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; source /opt/ros/jazzy/setup.bash"

# One ssh connection for the whole script (one password prompt).
_REMOTE_CTL="$(mktemp -u /tmp/remote_robot_ssh.XXXXXX)"
SSH_OPTS=(-o ControlMaster=auto -o ControlPath="$_REMOTE_CTL" -o ControlPersist=30 -o ConnectTimeout=8)
trap 'ssh "${SSH_OPTS[@]}" -O exit "$ROBOT" 2>/dev/null || true' EXIT

# remote <cmd...>       run a command on the robot
# remote "bash -s" <<EOF ... EOF   run a script on the robot (fed via stdin, so
#                       pgrep -f patterns in it never match the script itself)
remote() { ssh "${SSH_OPTS[@]}" "$ROBOT" "$@"; }
# remote_tty <cmd...>   interactive (tail -f, tmux attach)
remote_tty() { ssh -t "${SSH_OPTS[@]}" "$ROBOT" "$@"; }

# pgrep -f patterns for python programs. Anchored to a cmdline that *starts*
# with the python interpreter: a tmux server's own cmdline is the command of
# the first session it was created for (e.g. "tmux new-session ... python3 -u
# camera_test.py ..."), so an unanchored 'camera_test.py' also matches -- and
# kills -- the tmux server, taking every other session (robot_rgbd!) with it.
PAT_CAMERA_WEB='^[^ ]*python[0-9.]* .*camera_test\.py'
PAT_CAR2_WEB='^[^ ]*python[0-9.]* .*car2_web\.py'
PAT_MAP_WEB='^[^ ]*python[0-9.]* .*map_web\.py'

# Print the usage block at the top of the calling script (lines starting "# " after line 3).
usage() { sed -n '4,/^$/p' "$0" | sed 's/^# \{0,1\}//'; }
