#!/bin/bash
set -e
source /ws/install/setup.bash

usage() {
    echo "Usage: $0 [action ...]"
    echo "Accepted actions: play pause stop shutdown brake_release power_off"
    echo 'Examples:'
    echo "  $0 stop play"
    echo '  $0 "stop play"'
    echo '  $0 "brake release" "power off"'
}

if [[ $# -eq 0 ]]; then
    usage
    exit 1
fi

# Flatten quoted argument groups into words. This allows both:
#   ./ur_dashboard.sh stop play
#   ./ur_dashboard.sh "stop play"
words=()
for arg in "$@"; do
    read -r -a parts <<< "$arg"
    words+=("${parts[@]}")
done

# Convert aliases to the canonical dashboard action names.
actions=()
i=0
while (( i < ${#words[@]} )); do
    word=${words[i],,}
    word=${word//-/_}

    next=""
    if (( i + 1 < ${#words[@]} )); then
        next=${words[i+1],,}
        next=${next//-/_}
    fi

    case "$word $next" in
        "brake release")
            actions+=("brake_release")
            ((i += 2))
            ;;
        "power off")
            actions+=("power_off")
            ((i += 2))
            ;;
        *)
            case "$word" in
                play|pause|stop|shutdown|brake_release|power_off)
                    actions+=("$word")
                    ((i += 1))
                    ;;
                *)
                    echo "Error: unsupported action '$word'" >&2
                    echo "Accepted actions: play pause stop shutdown brake_release power_off" >&2
                    exit 1
                    ;;
            esac
            ;;
    esac
done

# All inputs are validated above. Execute each action on the left robot.
for action in "${actions[@]}"; do
    echo "Calling '$action' on left_ur..."
    ros2 service call \
        "/left_ur/dashboard_client/$action" \
        std_srvs/srv/Trigger

done

# All inputs are validated above. Execute each action on the right robot.
# for action in "${actions[@]}"; do
#     echo "Calling '$action' on right_ur..."
#     ros2 service call \
#         "/right_ur/dashboard_client/$action" \
#         std_srvs/srv/Trigger

# done

echo "All requested actions completed successfully."