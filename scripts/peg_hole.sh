#!/usr/bin/env bash

source /opt/ros/humble/setup.bash

pose_pub_pid=""
execute_pid=""
recording_active=false

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

cleanup_execute_call() {
    if [[ -n "$execute_pid" ]] && kill -0 "$execute_pid" 2>/dev/null; then
        echo
        echo "Interrupting visual-feedback insertion..."

        kill -INT "$execute_pid" 2>/dev/null

        # Give the ROS 2 process time to react to SIGINT.
        for _ in {1..30}; do
            if ! kill -0 "$execute_pid" 2>/dev/null; then
                break
            fi
            sleep 0.1
        done

        # Escalate if it did not stop.
        if kill -0 "$execute_pid" 2>/dev/null; then
            kill -TERM "$execute_pid" 2>/dev/null
        fi

        wait "$execute_pid" 2>/dev/null
    fi

    execute_pid=""
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
    cleanup_execute_call
    stop_recording_if_active

    exit "$exit_status"
}

handle_signal() {
    echo
    echo "Script interrupted."
    cleanup_execute_call
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
            "{header: {frame_id: 'world'}, pose: {position: {x: 0.759, y: 0.090, z: 0.128}, orientation: {x: 0.574, y: -0.563, z: 0.374, w: 0.462}}}" \
            >/dev/null

        # Avoid restarting ros2 topic pub too aggressively.
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
    read -r -p "Press Enter to start recording, or e to exit: " start_choice

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
echo "Starting visual-feedback insertion."
echo "Press e to interrupt it and stop recording."

ros2 service call \
    /visual_feedback_insertion/execute \
    std_srvs/srv/Trigger "{}" &

execute_pid=$!

visual_insertion_interrupted=false

while kill -0 "$execute_pid" 2>/dev/null; do
    if IFS= read -r -s -n 1 -t 0.1 key; then
        case "${key,,}" in
            e)
                visual_insertion_interrupted=true
                cleanup_execute_call
                break
                ;;
        esac
    fi
done

# Collect the exit status if the process completed normally.
execute_status=0

if [[ -n "$execute_pid" ]]; then
    wait "$execute_pid" || execute_status=$?
    execute_pid=""
fi

if [[ "$visual_insertion_interrupted" == true ]]; then
    echo "Visual-feedback insertion interrupted by the user."

    stop_recording_if_active
    restore_cartesian_controller

    ros2 service call /visual_feedback_insertion/stop std_srvs/srv/Trigger "{}"
    ros2 service call /discard_last_recording std_srvs/srv/Trigger "{}"

    echo "Recording stopped and Cartesian controller restored. Exiting."
    exit 0
fi

if (( execute_status != 0 )); then
    echo "Visual-feedback insertion failed with status ${execute_status}."

    stop_recording_if_active
    restore_cartesian_controller

    ros2 service call /discard_last_recording std_srvs/srv/Trigger "{}"

    echo "Recording stopped and Cartesian controller restored. Exiting."
    exit "$execute_status"
fi

echo "Visual-feedback insertion completed."

ros2 service call /pause_recording std_srvs/srv/Trigger "{}"

# ---------------------------------------------------------------------------
# Model-based insertion
# ---------------------------------------------------------------------------

while true; do
    read -r -p "Press Enter to continue with model-based insertion, or e to stop: " continue_choice

    case "${continue_choice,,}" in
        "")
            ros2 service call /resume_recording std_srvs/srv/Trigger "{}"
            break
            ;;
        e)
            stop_recording_if_active
            restore_cartesian_controller

            echo "Recording stopped. Exiting."
            exit 0
            ;;
        *)
            echo "Invalid choice. Press Enter or type e."
            ;;
    esac
done

ros2 control switch_controllers \
    --activate left_arm_controller \
    --deactivate left_cartesian_controller

# ---------------------------------------------------------------------------
# Stop recording
# ---------------------------------------------------------------------------

read -r -p "Press Enter to stop recording..."

stop_recording_if_active
restore_cartesian_controller

# ---------------------------------------------------------------------------
# Confirm or discard the last recording
# ---------------------------------------------------------------------------

while true; do
    read -r -p "Press Enter to confirm, or e to discard the last recording and exit: " confirm_choice

    case "${confirm_choice,,}" in
        "")
            echo "Alignment confirmed."
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