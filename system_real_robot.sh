#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd "$SCRIPT_DIR" || exit 1
source ./unitree_setup.sh || exit 1
source ./install/setup.bash || exit 1
ros2 launch vehicle_simulator system_real_robot.launch
