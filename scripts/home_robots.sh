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
#      joint_trajectory_controller and deactivate aikido_controller. The
#      switch is idempotent: the current controller states are queried first
#      and only the needed transitions are requested, because
#      switch_controllers --strict fails if a controller is already in the
#      desired state (re-running the same switch would abort the script).
#   2. ros2 topic pub ... "$(< FILE)"   -> publish the home trajectory, which is a
#      trajectory_msgs/msg/JointTrajectory built straight from a YAML file.
#      Publishing waits for the JTC's subscription (-w 1) and is bounded by
#      --pub-timeout, so a wrong topic cannot hang the script.
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
#
# NOTE: this targets the use_aikido:=true launch variant, whose controller set
# (config/aikido_controllers.yaml) contains 'aikido_controller' (active) and
# 'joint_trajectory_controller' (stopped). With the default controllers file
# the arm JTCs are named left/right_arm_controller instead and are always
# active, so the default --to/--from do not apply there.

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
PUB_TIMEOUT=30
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
      --pub-timeout SECS  Max time to wait for a subscriber when publishing
                        the trajectory. Default: $PUB_TIMEOUT
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
        --pub-timeout)     PUB_TIMEOUT="${2:?}"; shift 2 ;;
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

# Controller states as 'name state' lines ('' if the controller manager is
# not reachable; the query is bounded so a missing stack cannot hang us).
# States: active | inactive | stopped | unconfigured | error
list_ctrl_states() {
    timeout 5 ros2 control list_controllers "${CM[@]}" 2>/dev/null \
        | sed 's/\x1b\[[0-9;]*m//g'
}

ctrl_state() { # $1 = name, $2 = pre-fetched list (optional)
    printf '%s\n' "${2:-$(list_ctrl_states)}" | awk -v c="$1" '$1 == c { print $NF; exit }'
}

# Switch controllers idempotently: query the current states and only request
# the transitions that are needed. switch_controllers --strict fails if any
# controller is already in its requested state, so repeating the same switch
# (e.g. running the script twice in a row) would error out otherwise.
#   $1 = controller that must end up ACTIVE, $2 = one that must end up inactive
ensure_switch() {
    local target="$1" source="$2"
    local s_target s_source
    local activate=() deactivate=()

    local states
    states="$(list_ctrl_states)" || true  # CM unreachable: states stay empty
    s_target="$(ctrl_state "$target" "$states")"
    s_source="$(ctrl_state "$source" "$states")"
    # Offline dry-run: assume the canonical starting state so the full command
    # is printed even though the controller manager is not reachable.
    if $DRY_RUN; then
        [[ -n "$s_target" ]] || s_target='inactive'
        [[ -n "$s_source" ]] || s_source='active'
    fi

    case "$s_target" in
        active)            : ;;  # already where it should be
        inactive|stopped)  activate+=(--activate "$target") ;;
        error)             echo "Error: '$target' is in 'error' state; check the controller manager logs." >&2; exit 1 ;;
        "")                echo "Error: controller '$target' is not loaded (is the launch running?)." >&2; exit 1 ;;
        *)                 echo "Error: controller '$target' is '$s_target'; cannot activate it." >&2; exit 1 ;;
    esac
    case "$s_source" in
        active)            deactivate+=(--deactivate "$source") ;;
        inactive|stopped|unconfigured)
            echo "'$source' is not active ($s_source); skipping its deactivation." ;;
        error)             echo "Warning: '$source' is in 'error' state; not deactivating it." >&2 ;;
        "")                echo "Warning: controller '$source' is not loaded; skipping its deactivation." >&2 ;;
    esac

    if ((${#activate[@]} == 0 && ${#deactivate[@]} == 0)); then
        echo "'$target' is already active and '$source' is not; nothing to switch."
        return 0
    fi

    echo "Switching: ${deactivate[*]:-} ${activate[*]:-} ..."
    run ros2 control switch_controllers \
        --strict --spin-time 3 \
        ${deactivate[@]+"${deactivate[@]}"} \
        ${activate[@]+"${activate[@]}"} \
        "${CM[@]}"
}

switch_to_jtc() { ensure_switch "$TO_CTRL" "$FROM_CTRL"; }
switch_back()   { ensure_switch "$FROM_CTRL" "$TO_CTRL"; }

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
# -w 1 waits until the JTC's subscription is visible before publishing (it is
# also the default with --once, but explicit beats implicit), and the timeout
# bounds the wait so a wrong topic cannot hang the script.
echo "Publishing home trajectory to $TO_CTRL ..."
if $DRY_RUN; then
    run timeout "$PUB_TIMEOUT" ros2 topic pub --once -w 1 "$TOPIC" "$MSG_TYPE" "<content of $FILE>"
else
    if ! timeout "$PUB_TIMEOUT" ros2 topic pub --once -w 1 "$TOPIC" "$MSG_TYPE" "$(cat "$FILE")"; then
        echo "Error: publishing the trajectory failed (no matching subscriber within ${PUB_TIMEOUT}s, or malformed message)." >&2
        exit 1
    fi
fi

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
