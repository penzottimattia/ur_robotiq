#!/usr/bin/env bash

source /opt/ros/humble/setup.bash

pose_pub_pids=()
execute_pid=""
recording_active=false
post_mode=false
bimanual=false

usage() {
    echo "Usage: $0 [--post] [--bimanual]"
    echo "  default     Pause, switch controller, and resume recording."
    echo "  --post      Pause, wait 1 second, resume, and run interruptible execute_post."
    echo "  --bimanual  Use both left and right arms/controllers."
}

while (( $# > 0 )); do
    case "$1" in
        --post)
            post_mode=true
            ;;
        --bimanual)
            bimanual=true
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
    shift
done

# ---------------------------------------------------------------------------
# Controller helpers
# ---------------------------------------------------------------------------

restore_cartesian_controller() {
    if [[ "$bimanual" == true ]]; then
        ros2 control switch_controllers \
            --deactivate left_arm_controller right_arm_controller \
            --activate left_cartesian_controller right_cartesian_controller
    else
        ros2 control switch_controllers \
            --deactivate left_arm_controller \
            --activate left_cartesian_controller
    fi
}

activate_arm_controllers() {
    if [[ "$bimanual" == true ]]; then
        ros2 control switch_controllers \
            --activate left_arm_controller right_arm_controller \
            --deactivate left_cartesian_controller right_cartesian_controller
    else
        ros2 control switch_controllers \
            --activate left_arm_controller \
            --deactivate left_cartesian_controller
    fi
}

# ---------------------------------------------------------------------------
# Cleanup helpers
# ---------------------------------------------------------------------------

cleanup_pose_publisher() {
    for pid in "${pose_pub_pids[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null
            wait "$pid" 2>/dev/null
        fi
    done
    pose_pub_pids=()
}

cleanup_execute_call() {
    if [[ -n "$execute_pid" ]] && kill -0 "$execute_pid" 2>/dev/null; then
        echo
        echo "Interrupting visual-feedback insertion..."
        kill -INT "$execute_pid" 2>/dev/null

        for _ in {1..30}; do
            if ! kill -0 "$execute_pid" 2>/dev/null; then
                break
            fi
            sleep 0.1
        done

        if kill -0 "$execute_pid" 2>/dev/null; then
            kill -TERM "$execute_pid" 2>/dev/null
        fi

        wait "$execute_pid" 2>/dev/null
    fi

    execute_pid=""
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
# Pose publishers
# ---------------------------------------------------------------------------

start_pose_publishers() {
    (
        while true; do
            ros2 topic pub --once \
                /left_cartesian_controller/target_frame \
                geometry_msgs/msg/PoseStamped \
                "{header: {frame_id: 'world'}, pose: {position: {x: 0.680, y: 0.092, z: 0.222}, orientation: {x: 0.560, y: -0.580, z: 0.282, w: 0.520}}}" \
                >/dev/null
            sleep 0.1
        done
    ) &
    pose_pub_pids+=($!)

    if [[ "$bimanual" == true ]]; then
        (
            while true; do
                ros2 topic pub --once \
                    /right_cartesian_controller/target_frame \
                    geometry_msgs/msg/PoseStamped \
                    "{header: {frame_id: 'world'}, pose: {position: {x: 0.559, y: -0.076, z: 0.006}, orientation: {x: 0.441, y: 0.519, z: 0.540, w: 0.495}}}" \
                    >/dev/null
                sleep 0.1
            done
        ) &
        pose_pub_pids+=($!)
    fi
}

# ---------------------------------------------------------------------------
# Home position and alignment
# ---------------------------------------------------------------------------

echo "Publishing the home position until alignment is confirmed..."
start_pose_publishers

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
    exit 0
fi

if (( execute_status != 0 )); then
    echo "Visual-feedback insertion failed with status ${execute_status}."
    stop_recording_if_active
    restore_cartesian_controller
    ros2 service call /discard_last_recording std_srvs/srv/Trigger "{}"
    exit "$execute_status"
fi

echo "Visual-feedback insertion completed."

ros2 service call /pause_recording std_srvs/srv/Trigger "{}"

# ---------------------------------------------------------------------------
# Follow-up insertion mode
# ---------------------------------------------------------------------------

if [[ "$post_mode" == true ]]; then

    echo "Post mode: waiting 1 second before resuming recording..."
    sleep 1

    ros2 service call /resume_recording std_srvs/srv/Trigger "{}"

    echo
    echo "Starting visual-feedback post insertion."
    echo "Press e to interrupt it and stop recording."

    ros2 service call \
        /visual_feedback_insertion/execute_post \
        std_srvs/srv/Trigger "{}" &

    execute_pid=$!

    post_insertion_interrupted=false

    while kill -0 "$execute_pid" 2>/dev/null; do
        if IFS= read -r -s -n 1 -t 0.1 key; then
            case "${key,,}" in
                e)
                    post_insertion_interrupted=true
                    cleanup_execute_call
                    ros2 service call \
                        /visual_feedback_insertion/stop \
                        std_srvs/srv/Trigger "{}"
                    break
                    ;;
            esac
        fi
    done

    post_execute_status=0

    if [[ -n "$execute_pid" ]]; then
        wait "$execute_pid" || post_execute_status=$?
        execute_pid=""
    fi

    if [[ "$post_insertion_interrupted" == true ]]; then
        echo "Visual-feedback post insertion interrupted by the user."
        stop_recording_if_active
        restore_cartesian_controller
        exit 0
    fi

    if (( post_execute_status != 0 )); then
        echo "Visual-feedback post insertion failed with status ${post_execute_status}."
        stop_recording_if_active
        restore_cartesian_controller
        exit "$post_execute_status"
    fi

    echo "Visual-feedback post insertion completed."

else

    while true; do
        read -r -p "Press Enter to continue with model-based insertion, or e to stop: " continue_choice

        case "${continue_choice,,}" in
            "")
                activate_arm_controllers
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
fi

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
            echo "Recording confirmed."
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