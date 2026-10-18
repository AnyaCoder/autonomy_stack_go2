#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd "$SCRIPT_DIR" || exit 1
# shellcheck source=/dev/null
source ./install/setup.bash
export ROS_DOMAIN_ID="${GO2_SIM_DOMAIN_ID:-94}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" multicast="true" /></Interfaces></General></Domain></CycloneDDS>'
./src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64 &
unity_pid=$!
trap 'kill "$unity_pid" 2>/dev/null || true' EXIT
sleep 3 
ros2 launch vehicle_simulator system_simulation_with_route_planner.launch
