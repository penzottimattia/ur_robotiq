#!/bin/bash

set -e

source /ws/install/setup.bash

if [[ $# -eq 0 ]]; then
    echo "Usage: $0 <action> [action ...]"
    echo "Accepted actions: play stop shutdown brake_release power_off"
    exit 1
fi

# Validate all arguments before calling any service.
for action in "$@"; do
    case "$action" in
        play|stop|shutdown|brake_release|power_off)
            ;;
        *)
            echo "Error: unsupported action '$action'" >&2
            echo "Accepted actions: play stop shutdown brake_release power_off" >&2
            exit 1
            ;;
    esac
done

# Execute each action on the left robot first, then on the right robot.
for action in "$@"; do
    echo "Calling '$action' on left_ur..."
    ros2 service call \
        "/left_ur/dashboard_client/$action" \
        std_srvs/srv/Trigger

    echo "Calling '$action' on right_ur..."
    ros2 service call \
        "/right_ur/dashboard_client/$action" \
        std_srvs/srv/Trigger
done

echo "All requested actions completed successfully."