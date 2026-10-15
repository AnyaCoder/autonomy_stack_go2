#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd "$SCRIPT_DIR" || exit 1
# shellcheck disable=SC1091
source ./unitree_setup.sh || exit 1
# shellcheck disable=SC1091
source ./install/setup.bash || exit 1
exec python3 "$SCRIPT_DIR/tools/ros_process_group.py" --replace ros2 launch vehicle_simulator system_real_robot_with_route_planner.launch "$@"
