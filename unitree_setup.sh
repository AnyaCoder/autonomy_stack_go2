#!/bin/bash
echo "Setup unitree ros2 environment"
source /opt/ros/foxy/setup.bash || return 1

# Go2 uses DDS domain 0 on the wired interface. Both can be overridden.
export ROS_DOMAIN_ID="${GO2_ROS_DOMAIN_ID:-0}"
GO2_NETWORK_INTERFACE="${GO2_NETWORK_INTERFACE:-enp2s0}"
IFS=',' read -r -a GO2_DDS_INTERFACES <<< "$GO2_NETWORK_INTERFACE"
GO2_CYCLONEDDS_INTERFACES=""
for GO2_DDS_INTERFACE in "${GO2_DDS_INTERFACES[@]}"; do
    if [[ ! -d "/sys/class/net/$GO2_DDS_INTERFACE" ]]; then
        echo "Network interface $GO2_DDS_INTERFACE does not exist; set GO2_NETWORK_INTERFACE." >&2
        return 1
    fi
    GO2_CYCLONEDDS_INTERFACES+="<NetworkInterface name=\"$GO2_DDS_INTERFACE\" priority=\"default\" multicast=\"true\" />"
done

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
GO2_CYCLONEDDS_PREFIX="${GO2_CYCLONEDDS_PREFIX:-$HOME/cyclonedds-0.10.2/install}"
if [[ -f "$GO2_CYCLONEDDS_PREFIX/lib/libddsc.so" ]]; then
    export LD_LIBRARY_PATH="$GO2_CYCLONEDDS_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    GO2_CYCLONEDDS_PEERS=""
    if [[ -n "${GO2_DDS_PEER:-}" ]]; then
        GO2_CYCLONEDDS_PEERS="<Discovery><Peers><Peer Address=\"$GO2_DDS_PEER\" /></Peers></Discovery>"
    fi
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>$GO2_CYCLONEDDS_INTERFACES</Interfaces></General>$GO2_CYCLONEDDS_PEERS</Domain></CycloneDDS>"
else
    echo "Cyclone DDS 0.10.2 is missing at $GO2_CYCLONEDDS_PREFIX; set GO2_CYCLONEDDS_PREFIX." >&2
    return 1
fi
