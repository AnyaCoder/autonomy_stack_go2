#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage: go2-services.sh [status|start|stop|restart|enable|disable|logs] [all|wireless|sensors]

Groups:
  all       All seven Go2 user services (default)
  wireless  native DDS relay and ROS graph announcer as one stack
  sensors   Lidar observer, front camera, lowstate, RealSense, preview, and laserscan
EOF
}

action="${1:-status}"
group="${2:-all}"

case "$group" in
    all)
        units=(
            go2-wifi-stack.target
            go2-sensors.service
            go2-front-camera.service
            go2-lowstate.service
            go2-realsense.service
            go2-realsense-preview.service
            go2-laserscan.service
        )
        ;;
    wireless)
        units=(go2-wifi-stack.target)
        ;;
    sensors)
        units=(
            go2-sensors.service
            go2-front-camera.service
            go2-lowstate.service
            go2-realsense.service
            go2-realsense-preview.service
            go2-laserscan.service
        )
        ;;
    -h|--help|help)
        usage
        exit 0
        ;;
    *)
        usage >&2
        exit 2
        ;;
esac

case "$action" in
    status|start|stop|restart|enable|disable)
        systemctl --user --no-pager "$action" "${units[@]}"
        ;;
    logs)
        log_args=()
        for unit in "${units[@]}"; do
            if [[ "$unit" == go2-wifi-stack.target ]]; then
                log_args+=(--unit go2-wifi-relay.service --unit go2-wifi-relay-wifi.service --unit go2-wifi-graph-announcer.service)
            else
                log_args+=(--unit "$unit")
            fi
        done
        journalctl --user --no-pager --lines=80 "${log_args[@]}"
        ;;
    -h|--help|help)
        usage
        ;;
    *)
        usage >&2
        exit 2
        ;;
esac
