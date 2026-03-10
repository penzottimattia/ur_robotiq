#!/usr/bin/env bash
set -euo pipefail

MODE="${MODE:-mock_grippers_only}"
LEFT_ROBOT_IP="${LEFT_ROBOT_IP:-192.168.0.10}"
RIGHT_ROBOT_IP="${RIGHT_ROBOT_IP:-192.168.0.11}"
LEFT_PROGRAM="${LEFT_PROGRAM:-ExternalControl.urp}"
RIGHT_PROGRAM="${RIGHT_PROGRAM:-ExternalControl.urp}"
DASHBOARD_TIMEOUT="${DASHBOARD_TIMEOUT:-20.0}"
BASE_POSES_FILE="${BASE_POSES_FILE:-}"
CONTROLLERS_FILE="${CONTROLLERS_FILE:-}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      MODE="$2"
      shift 2
      ;;
    --left-robot-ip)
      LEFT_ROBOT_IP="$2"
      shift 2
      ;;
    --right-robot-ip)
      RIGHT_ROBOT_IP="$2"
      shift 2
      ;;
    --left-program)
      LEFT_PROGRAM="$2"
      shift 2
      ;;
    --right-program)
      RIGHT_PROGRAM="$2"
      shift 2
      ;;
    --dashboard-timeout)
      DASHBOARD_TIMEOUT="$2"
      shift 2
      ;;
    --base-poses-file)
      BASE_POSES_FILE="$2"
      shift 2
      ;;
    --controllers-file)
      CONTROLLERS_FILE="$2"
      shift 2
      ;;
    -h|--help)
      cat <<'EOF'
Start bimanual control and run UR setup sequence (power on, brake release, load/play).

Usage:
  start_bimanual_control.sh [options]

Options:
  --mode <full_mock|mock_grippers_only>   Control mode (default: mock_grippers_only)
  --left-robot-ip <ip>                    Left UR robot IP (default: 192.168.0.10)
  --right-robot-ip <ip>                   Right UR robot IP (default: 192.168.0.11)
  --left-program <path|name>              Left program to load (default: ExternalControl.urp)
  --right-program <path|name>             Right program to load (default: ExternalControl.urp)
  --dashboard-timeout <seconds>           Dashboard timeout (default: 20.0)
  --base-poses-file <path>                Optional base poses yaml override
  --controllers-file <path>               Optional controllers yaml override
  -h, --help                              Show this help

Environment overrides are also supported:
  MODE, LEFT_ROBOT_IP, RIGHT_ROBOT_IP, LEFT_PROGRAM, RIGHT_PROGRAM,
  DASHBOARD_TIMEOUT, BASE_POSES_FILE, CONTROLLERS_FILE
EOF
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

launch_args=(
  mode:="$MODE"
  left_robot_ip:="$LEFT_ROBOT_IP"
  right_robot_ip:="$RIGHT_ROBOT_IP"
  launch_dashboard_clients:=true
  dashboard_receive_timeout:="$DASHBOARD_TIMEOUT"
  run_setup_node:=true
  left_program:="$LEFT_PROGRAM"
  right_program:="$RIGHT_PROGRAM"
)

if [[ -n "$BASE_POSES_FILE" ]]; then
  launch_args+=(base_poses_file:="$BASE_POSES_FILE")
fi

if [[ -n "$CONTROLLERS_FILE" ]]; then
  launch_args+=(controllers_file:="$CONTROLLERS_FILE")
fi

echo "Starting bimanual control with setup sequence..."
exec ros2 launch ur_robotiq control_bimanual_ur3_robotiq.launch.py "${launch_args[@]}"
