#!/usr/bin/env bash

source /opt/ros/humble/setup.bash

pose_pub_pid=""
recording_active=false
insertion_active=false

# ---------------------------------------------------------------------------
# Cleanup helpers
# ---------------------------------------------------------------------------

cleanup_pose_publisher() {
    if [[ -n "$pose_pub_pid" ]] && kill -0 "$pose_pub_pid" 2>/dev/null; then
        kill "$pose_pub_pid" 2>/dev/null
        wait "$pose_pub_pid" 2>/dev/null
    fi

    pose_pub_pid=""
}

stop_insertion_if_active() {
    if [[ "$insertion_active" == true ]]; then
        echo "Stopping visual-feedback insertion..."
        ros2 service call /visual_feedback_insertion/stop std_srvs/srv/Trigger "{}"
        insertion_active=false
    fi
}

restore_cartesian_controller() {
    ros2 control switch_controllers \
        --deactivate left_arm_controller \
        --activate left_cartesian_controller
}

stop_recording_if_active() {
    if [[ "$recording_active" == true ]]; then
        echo "Stopping recording..."
        ros2 service call /stop_recording std_srvs/srv/Trigger "{}"
        recording_active=false
    fi
}

cleanup() {
    local exit_status=$?

    trap - EXIT INT TERM

    cleanup_pose_publisher
    stop_insertion_if_active
    stop_recording_if_active

    exit "$exit_status"
}

handle_signal() {
    echo
    echo "Script interrupted."
    cleanup_pose_publisher
    stop_insertion_if_active
    stop_recording_if_active
    restore_cartesian_controller
    exit 130
}

trap cleanup EXIT
trap handle_signal INT TERM

# ---------------------------------------------------------------------------
# Home position and alignment
# ---------------------------------------------------------------------------

echo "Publishing the home position until alignment is confirmed..."

(
    while true; do
        ros2 topic pub --once \
            /left_cartesian_controller/target_frame \
            geometry_msgs/msg/PoseStamped \
            "{header: {frame_id: 'world'}, pose: {position: {x: 0.759, y: 0.090, z: 0.118}, orientation: {x: 0.574, y: -0.563, z: 0.374, w: 0.462}}}" \
            >/dev/null
        sleep 0.1
    done
) &

pose_pub_pid=$!

while true; do
    read -r -p "Press Enter when alignment is confirmed, or e to exit: " alignment_choice

    case "${alignment_choice,,}" in
        "")
            echo "Alignment confirmed."
            break
            ;;
        e)
            echo "Exiting."
            exit 0
            ;;
        *)
            echo "Invalid choice. Press Enter to confirm or type e to exit."
            ;;
    esac
done

cleanup_pose_publisher

# ---------------------------------------------------------------------------
# Visual-feedback insertion
# ---------------------------------------------------------------------------

while true; do
    read -r -p "Press Enter to start recording and insertion, or e to exit: " start_choice

    case "${start_choice,,}" in
        "")
            break
            ;;
        e)
            exit 0
            ;;
        *)
            echo "Invalid choice. Press Enter or type e."
            ;;
    esac
done

ros2 service call /start_recording std_srvs/srv/Trigger "{}"
recording_active=true

echo
echo "Starting visual-feedback insertion..."
ros2 service call /visual_feedback_insertion/start std_srvs/srv/Trigger "{}"
insertion_active=true

echo
read -r -p "Press Enter to stop the insertion..."

stop_insertion_if_active
stop_recording_if_active
restore_cartesian_controller

echo "Visual-feedback insertion and recording stopped."

# ---------------------------------------------------------------------------
# Confirm or discard the last recording
# ---------------------------------------------------------------------------

while true; do
    read -r -p "Press Enter to confirm the last recording, or e to discard it and exit: " confirm_choice

    case "${confirm_choice,,}" in
        "")
            echo "Last recording confirmed."
            break
            ;;
        e)
            echo "Discarding the last recording..."
            ros2 service call /discard_last_recording std_srvs/srv/Trigger "{}"
            echo "Last recording discarded. Exiting."
            exit 0
            ;;
        *)
            echo "Invalid choice. Press Enter to confirm or type e to discard and exit."
            ;;
    esac
done