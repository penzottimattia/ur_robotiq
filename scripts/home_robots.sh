#!/usr/bin/env bash
#
# home_robots.sh -- home the bimanual UR + MIA robot using only the CLI.
# (The ROS environment is sourced before enabling `set -u`, because the
# workspace setup.bash references COLCON_TRACE which is unset by default.)
#
# It does two things, exactly like the old ROS1 "switch controllers + rostopic pub
# from a file" recipe, but with ROS 2 commands:
#
#   1. ros2 control switch_controllers  -> activate the (stopped)
#      joint_trajectory_controller and deactivate aikido_controller
#   2. ros2 topic pub ... "$(< FILE)"   -> publish the home trajectory, which is a
#      trajectory_msgs/msg/JointTrajectory built straight from a YAML file.
#
# The JTC interpolates to the file's target and then HOLDS the final point, so a
# single publish is enough to drive the robot home and keep it there. By default
# the script switches back to aikido_controller afterwards (re-engaging the safety
# controller); the robot stays at the home position because the (sim) hardware
# retains its joint state.
#
# The trajectory file is a partial JointTrajectory message; only joint_names,
# points[].positions and points[].time_from_start are set -- ros2 topic pub leaves
# the rest (header, velocities, accelerations, effort) at their defaults.

# ---------------------------------------------------------------------------
# ROS environment (sourced first; strict mode is enabled afterwards so that the
# setup.bash's own `COLCON_TRACE` reference does not trip `set -u`)
# ---------------------------------------------------------------------------
if [[ -f /ws/install/setup.bash ]]; then
    source /ws/install/setup.bash
else
    source /opt/ros/humble/setup.bash
fi

set -euo pipefail

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

TOPIC="/joint_trajectory_controller/joint_trajectory"
MSG_TYPE="trajectory_msgs/msg/JointTrajectory"
TO_CTRL="joint_trajectory_controller"   # controller we activate to home
FROM_CTRL="aikido_controller"           # controller we deactivate while homing
SWITCH_BACK=true
DO_SWITCH=true
SETTLE=3
DRY_RUN=false
FILE=""

# Resolve the default trajectory file: prefer the one next to this script
# (source tree), fall back to the installed package config.
default_file() {
    local candidate
    for candidate in \
        "$SCRIPT_DIR/../config/home_trajectory.yaml" \
        "$(ros2 pkg prefix ur_robotiq 2>/dev/null)/config/home_trajectory.yaml"; do
        [[ -f "$candidate" ]] && { echo "$candidate"; return 0; }
    done
    echo ""
}

usage() {
    cat <<EOF
Usage: $(basename "$0") [options]

Home the robot by activating the joint trajectory controller and publishing a
JointTrajectory from a YAML file.

Options:
  -f, --file FILE       Trajectory YAML file (a partial JointTrajectory message).
                        Default: config/home_trajectory.yaml next to this script.
      --topic TOPIC     JTC trajectory topic.
                        Default: $TOPIC
      --to CTRL         Controller to activate while homing.
                        Default: $TO_CTRL
      --from CTRL       Controller to deactivate while homing.
                        Default: $FROM_CTRL
      --settle SECS     Margin (seconds) added to the trajectory duration when
                        waiting for the move to complete (the JTC holds the
                        final point afterwards). Default: $SETTLE
      --no-switch-back  Leave '$TO_CTRL' active at the end (do not re-engage
                        '$FROM_CTRL').
      --no-switch       Do not switch controllers; assume '$TO_CTRL' is already
                        active.
      --dry-run         Print the commands instead of running them.
  -h, --help            Show this help.

Example:
    $(basename "$0")
    $(basename "$0") --file ~/my_home.yaml --settle 8
    $(basename "$0") --no-switch-back        # stay on the JTC at home
EOF
}

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
while (( $# > 0 )); do
    case "$1" in
        -f|--file)         FILE="${2:?}"; shift 2 ;;
        --topic)           TOPIC="${2:?}"; shift 2 ;;
        --to)              TO_CTRL="${2:?}"; shift 2 ;;
        --from)            FROM_CTRL="${2:?}"; shift 2 ;;
        --settle)          SETTLE="${2:?}"; shift 2 ;;
        --switch-back)     SWITCH_BACK=true; shift ;;
        --no-switch-back)  SWITCH_BACK=false; shift ;;
        --no-switch)       DO_SWITCH=false; shift ;;
        --dry-run)         DRY_RUN=true; shift ;;
        -h|--help)         usage; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ -z "$FILE" ]]; then
    FILE="$(default_file)"
    if [[ -z "$FILE" || ! -f "$FILE" ]]; then
        echo "Error: could not find a default trajectory file." >&2
        echo "Pass one explicitly with --file." >&2
        exit 1
    fi
fi
if [[ ! -f "$FILE" ]]; then
    echo "Error: trajectory file not found: $FILE" >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
run() {
    if $DRY_RUN; then
        local a
        printf 'DRY-RUN: '
        for a in "$@"; do printf '%q ' "$a"; done
        echo
    else
        "$@"
    fi
}

CM=( --controller-manager /controller_manager )

switch_to_jtc() {
    echo "Switching: deactivate '$FROM_CTRL', activate '$TO_CTRL' ..."
    run ros2 control switch_controllers \
        --strict --spin-time 3 \
        --deactivate "$FROM_CTRL" \
        --activate "$TO_CTRL" \
        "${CM[@]}"
}

switch_back() {
    echo "Switching back: deactivate '$TO_CTRL', activate '$FROM_CTRL' ..."
    run ros2 control switch_controllers \
        --strict --spin-time 3 \
        --deactivate "$TO_CTRL" \
        --activate "$FROM_CTRL" \
        "${CM[@]}"
}

# ---------------------------------------------------------------------------
# Preflight
# ---------------------------------------------------------------------------
echo "Home trajectory file : $FILE"
echo "Publish to           : $TOPIC ($MSG_TYPE)"
echo "Controllers          : $FROM_CTRL -> $TO_CTRL$([[ $SWITCH_BACK == true ]] && echo " -> $FROM_CTRL")"

actual_type="$(ros2 topic type "$TOPIC" 2>/dev/null || true)"
if [[ -n "$actual_type" && "$actual_type" != "$MSG_TYPE" ]]; then
    echo "Warning: $TOPIC is '$actual_type', expected '$MSG_TYPE'." >&2
fi

# Best-effort sanity check that the file parses as YAML (cheap, catches typos),
# and extract the trajectory duration (max time_from_start over all points).
TRAJ_SECS="$(python3 -c "
import sys, yaml
doc = yaml.safe_load(open('$FILE'))
secs = 0.0
for p in (doc or {}).get('points', []):
    t = p.get('time_from_start') or {}
    if isinstance(t, dict):
        secs = max(secs, float(t.get('sec', 0)) + float(t.get('nanosec', 0)) * 1e-9)
    else:
        secs = max(secs, float(t))
print(f'{secs:.3f}')
" 2>/dev/null || { echo "Error: $FILE is not valid YAML (or has no parseable points)." >&2; exit 1; })"
WAIT="$(awk -v d="$TRAJ_SECS" -v s="$SETTLE" 'BEGIN{printf "%.1f", d + s}')"
echo "Trajectory duration    : ${TRAJ_SECS}s (+ ${SETTLE}s settle margin = ${WAIT}s wait)"

# ---------------------------------------------------------------------------
# Switch controllers
# ---------------------------------------------------------------------------
if $DO_SWITCH; then
    switch_to_jtc
fi

# ---------------------------------------------------------------------------
# Publish the home trajectory (from the file)
# ---------------------------------------------------------------------------
# The file content is passed as a single argument; ros2 topic pub does
# yaml.safe_load() on it and maps the keys onto the JointTrajectory fields.
echo "Publishing home trajectory to $TO_CTRL ..."
run ros2 topic pub --once "$TOPIC" "$MSG_TYPE" "$(cat "$FILE")"

# Let the move complete: the JTC interpolates for the trajectory's own
# duration (the max time_from_start in the file) and holds the final point
# afterwards, so we wait that long plus a settle margin.
if $DRY_RUN; then
    echo "DRY-RUN: sleep $WAIT"
else
    echo "Waiting ${WAIT}s for the move to complete (${TRAJ_SECS}s trajectory + ${SETTLE}s settle; robot holds at home) ..."
    sleep "$WAIT"
fi

# ---------------------------------------------------------------------------
# Re-engage the safety controller by default
# ---------------------------------------------------------------------------
if $DO_SWITCH && $SWITCH_BACK; then
    switch_back
    echo "Done. Robot is held at home by $TO_CTRL and $FROM_CTRL is re-activated."
else
    echo "Done. Robot is held at home by $TO_CTRL."
fi
