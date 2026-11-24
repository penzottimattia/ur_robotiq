#!/bin/bash
set -e
source /ws/install/setup.bash

# Define color codes
RED='\e[31m'
BLUE='\e[34m'
GREEN='\e[32m'
BOLD='\e[1m'
RESET='\e[0m'

pause() {
  read -rp "-- Press [Enter] to continue..." _
}

echo -e "${BOLD}Open${RESET} the hand"
pause
ros2 param set /left_senseglove_interface calibration_opened true
ros2 param set /right_senseglove_interface calibration_opened true

echo -e "${BOLD}Close${RESET} the hand"
pause
ros2 param set /left_senseglove_interface calibration_closed true
ros2 param set /right_senseglove_interface calibration_closed true
