#!/usr/bin/env bash

set -eo pipefail

GO2_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
GO2_WIRED_INTERFACE="${GO2_WIRED_INTERFACE:-enp2s0}"
GO2_WIFI_INTERFACE="${GO2_WIFI_INTERFACE:-wlp4s0}"
GO2_WIFI_SSID="${GO2_WIFI_SSID:-Go2-Explore}"
GO2_WIFI_ADDRESS="${GO2_WIFI_ADDRESS:-192.168.76.2/24}"
GO2_FORCE_WIFI="${GO2_FORCE_WIFI:-0}"
GO2_SELECTED_INTERFACE=""

usage() {
    cat <<EOF
用法: ./go2.sh [connect|network|check|diagnose|build|calibrate-imu|slam|maps|map-merge|map-view|run|disconnect]
  connect        自动选择链路；可用 GO2_FORCE_WIFI=1 强制使用 Go2 Wi-Fi
  network        显示当前网络，不切换连接
  check          自动选择链路并检查 Domain 0 传感器话题
  diagnose       打印网络和 ROS 2 发现诊断，不切换网络
  build          编译工作区
  calibrate-imu  执行 L1 IMU 标定
  slam           启动独立 SLAM + RViz
  maps           列出已保存的建图批次
  map-merge      合并批次中的 PCD 分块，可选批次名，默认 latest
  map-view       在 RViz 查看批次地图，可选批次名，默认 latest
  run            确认后启动 README 实机系统
  disconnect     仅断开 Go2 Wi-Fi 配置，不影响有线网

网络默认值: 有线 $GO2_WIRED_INTERFACE, Wi-Fi $GO2_WIFI_INTERFACE ($GO2_WIFI_SSID)
指定 GO2_FORCE_WIFI=1 可在有线仍连接时强制用 Go2 Wi-Fi 检查/运行。
NetworkManager 必须已有保存的 Go2-Explore 配置；本脚本不会询问或保存密码。
EOF
}

require_network_tools() {
    local tool
    for tool in ip iw ping nmcli timeout; do
        if ! command -v "$tool" >/dev/null 2>&1; then
            echo "缺少命令: $tool" >&2
            return 1
        fi
    done
}

is_go2_wired_reachable() {
    [[ -d "/sys/class/net/$GO2_WIRED_INTERFACE" ]] || return 1
    ip -4 -o addr show dev "$GO2_WIRED_INTERFACE" | grep -q '192\.168\.123\.' || return 1
    ping -c 1 -W 2 -I "$GO2_WIRED_INTERFACE" 192.168.123.18 >/dev/null 2>&1
}

wired_carrier_up() {
    local carrier_file="/sys/class/net/$GO2_WIRED_INTERFACE/carrier"
    [[ -r "$carrier_file" && "$(<"$carrier_file")" == 1 ]]
}

selected_link_up() {
    if [[ "$GO2_SELECTED_INTERFACE" == "$GO2_WIRED_INTERFACE" ]]; then
        wired_carrier_up
    else
        is_go2_wifi_connected
    fi
}

is_go2_wifi_connected() {
    local link address
    [[ -d "/sys/class/net/$GO2_WIFI_INTERFACE" ]] || return 1
    link="$(iw dev "$GO2_WIFI_INTERFACE" link 2>/dev/null)" || return 1
    grep -Fq "SSID: $GO2_WIFI_SSID" <<<"$link" || return 1
    address="$(ip -4 -o addr show dev "$GO2_WIFI_INTERFACE" | awk '{print $4}')"
    grep -Fxq "$GO2_WIFI_ADDRESS" <<<"$address" || return 1
    # The Go2 AP may not answer ICMP; DDS discovery is the end-to-end check.
}

connect_go2_wifi() {
    require_network_tools || return 1
    [[ -d "/sys/class/net/$GO2_WIFI_INTERFACE" ]] || {
        echo "未找到 Wi-Fi 网卡 $GO2_WIFI_INTERFACE。" >&2
        return 1
    }
    if ! nmcli -t -f NAME connection show | grep -Fxq "$GO2_WIFI_SSID"; then
        echo "NetworkManager 没有保存 $GO2_WIFI_SSID 配置。请先在 Ubuntu 网络设置中保存该连接；go2.sh 不会询问密码。" >&2
        return 1
    fi
    echo "配置 $GO2_WIFI_SSID 使用本地地址 $GO2_WIFI_ADDRESS（不设置默认网关）..."
    nmcli connection modify id "$GO2_WIFI_SSID" \
        ipv4.method manual \
        ipv4.addresses "$GO2_WIFI_ADDRESS" \
        ipv4.gateway "" \
        ipv4.dns "" \
        ipv4.never-default yes \
        ipv6.method disabled
    echo "连接已保存的 $GO2_WIFI_SSID 配置（不询问密码，也不访问互联网）..."
    timeout 30s nmcli connection up id "$GO2_WIFI_SSID" ifname "$GO2_WIFI_INTERFACE"
}

detect_go2_network() {
    if [[ "$GO2_FORCE_WIFI" == "1" ]]; then
        if ! is_go2_wifi_connected; then
            GO2_SELECTED_INTERFACE=""
            return 1
        fi
        GO2_SELECTED_INTERFACE="$GO2_WIFI_INTERFACE"
        echo "Go2 链路: Wi-Fi ($GO2_SELECTED_INTERFACE, 强制选择)"
    elif is_go2_wired_reachable; then
        GO2_SELECTED_INTERFACE="$GO2_WIRED_INTERFACE"
        echo "Go2 链路: 有线 ($GO2_SELECTED_INTERFACE)"
    elif is_go2_wifi_connected; then
        GO2_SELECTED_INTERFACE="$GO2_WIFI_INTERFACE"
        echo "Go2 链路: Wi-Fi ($GO2_SELECTED_INTERFACE)"
    else
        GO2_SELECTED_INTERFACE=""
        return 1
    fi
    export GO2_NETWORK_INTERFACE="$GO2_SELECTED_INTERFACE"
}

select_go2_network() {
    require_network_tools || return 1
    if [[ "$GO2_FORCE_WIFI" == "1" ]]; then
        if ! is_go2_wifi_connected; then
            connect_go2_wifi || {
                echo '无法激活已保存的 Go2 Wi-Fi 配置。' >&2
                return 1
            }
        fi
        detect_go2_network || {
            echo 'Go2 Wi-Fi 未连接或地址配置不正确。' >&2
            return 1
        }
        return 0
    fi
    detect_go2_network && return 0
    connect_go2_wifi || {
        echo '有线 Go2 不可达，且无法激活已保存的 Go2 Wi-Fi 配置。' >&2
        return 1
    }
    detect_go2_network || {
        echo '已连接 Wi-Fi，但未获得 Go2-Explore 地址或热点不可达。' >&2
        return 1
    }
}

show_network() {
    require_network_tools || return 1
    nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status
    ip -4 -brief address show dev "$GO2_WIRED_INTERFACE" 2>/dev/null || true
    ip -4 -brief address show dev "$GO2_WIFI_INTERFACE" 2>/dev/null || true
    if is_go2_wired_reachable && is_go2_wifi_connected; then
        if [[ "$GO2_FORCE_WIFI" == "1" ]]; then
            echo '有线和 Go2 Wi-Fi 均可达；GO2_FORCE_WIFI=1 将选择 Wi-Fi。'
        else
            echo '有线可达；Go2 Wi-Fi 已关联并配置地址。'
        fi
    elif is_go2_wired_reachable; then
        echo "Go2 当前可经有线网卡 $GO2_WIRED_INTERFACE 到达。"
    elif is_go2_wifi_connected; then
        echo "Go2 Wi-Fi 已关联并配置地址；话题是否可用以 DDS 检查为准。"
    else
        echo '当前没有可达的 Go2 链路。运行 ./go2.sh connect 自动选择有线或 Wi-Fi。'
    fi
}

diagnostics() {
    require_network_tools || return 1
    echo '网络状态:'
    show_network || true
    echo '到 Go2 的路由:'
    ip -4 route get 192.168.123.18 2>&1 || true
    ip -4 route get 192.168.76.1 2>&1 || true
    if ! detect_go2_network; then
        echo '没有现成可达的 Go2 链路；诊断不会自动切换 Wi-Fi。'
        return 0
    fi
    if [[ "$GO2_SELECTED_INTERFACE" == "$GO2_WIFI_INTERFACE" ]]; then
        echo "Go2 热点地址检查 (192.168.76.1, ICMP 仅供参考):"
        if ! ping -c 1 -W 2 -I "$GO2_SELECTED_INTERFACE" 192.168.76.1; then
            echo '未收到 ICMP 回复；继续检查邻居表和 DDS，热点可能屏蔽 ping。'
        fi
        ip neigh show to 192.168.76.1 dev "$GO2_SELECTED_INTERFACE" || true
    fi
    source_robot_env || return 1
    reset_ros_cli_graph
    echo "ROS 2 话题列表（接口 $GO2_SELECTED_INTERFACE，最多等待 8 秒）:"
    timeout 8s ros2 topic list 2>&1 || true
}

source_robot_env() {
    export GO2_NETWORK_INTERFACE="$GO2_SELECTED_INTERFACE"
    export ROS_LOCALHOST_ONLY=0
    if [[ ",$GO2_SELECTED_INTERFACE," == *",$GO2_WIFI_INTERFACE,"* ]]; then
        export GO2_DDS_PEER="${GO2_DDS_PEER:-192.168.76.1}"
    else
        unset GO2_DDS_PEER
    fi
    # shellcheck disable=SC1090
    source "$GO2_ROOT/unitree_setup.sh" || return 1
    if [[ ! -f "$GO2_ROOT/install/setup.bash" ]]; then
        echo "工作区尚未编译，请先运行: ./go2.sh build" >&2
        return 1
    fi
    # shellcheck disable=SC1090
    source "$GO2_ROOT/install/setup.bash"
}

report_wireless_discovery_state() {
    [[ "$GO2_SELECTED_INTERFACE" == "$GO2_WIFI_INTERFACE" ]] || return 0
    echo '无线 DDS 发现失败时的链路状态:' >&2
    iw dev "$GO2_WIFI_INTERFACE" link >&2 || true
    ip -4 -brief address show dev "$GO2_WIFI_INTERFACE" >&2 || true
    ip neigh show 192.168.76.1 dev "$GO2_WIFI_INTERFACE" >&2 || true
    echo '继续检查热点关联、DDS 发现和狗端 relay；以电脑实际收到样本为准。' >&2
}

topic_info() {
    local topic="$1" endpoint="$2" info count attempt
    attempt=0
    while (( attempt < 10 )); do
        attempt=$((attempt + 1))
        info="$(ros2 topic info "$topic" --verbose 2>&1 || true)"
        count="$(awk -F': ' -v endpoint="$endpoint" '$1 == endpoint {print $2}' <<<"$info")"
        if [[ "$count" =~ ^[1-9][0-9]*$ ]]; then
            printf '%s\n' "$info"
            return 0
        fi
        (( attempt == 10 )) || sleep 1
    done
    printf '等待 10 秒后话题端点仍未出现: %s (%s)\n' "$topic" "$endpoint" >&2
    [[ -z "$info" ]] || printf '%s\n' "$info" >&2
    report_wireless_discovery_state
    return 1
}

topic_sample() {
    local topic="$1" sample_file baseline_info baseline_count info count
    local sample_attempt=0 subscriber_pid
    sample_file="$(mktemp)" || return 1
    baseline_info="$(ros2 topic info "$topic" --verbose 2>&1 || true)"
    baseline_count="$(awk -F': ' '$1 == "Subscription count" {print $2}' <<<"$baseline_info")"
    [[ "$baseline_count" =~ ^[0-9]+$ ]] || baseline_count=0
    printf '电脑已有订阅者（不含 check 探针）: %s\n' "$baseline_count"

    PYTHONUNBUFFERED=1 ros2 topic echo "$topic" --qos-profile sensor_data --no-arr \
        >"$sample_file" 2>/dev/null &
    subscriber_pid=$!

    while (( sample_attempt < 16 )) && [[ ! -s "$sample_file" ]]; do
        sleep 0.5
        sample_attempt=$((sample_attempt + 1))
    done

    if [[ ! -s "$sample_file" ]]; then
        info="$(ros2 topic info "$topic" --verbose 2>&1 || true)"
        count="$(awk -F': ' '$1 == "Subscription count" {print $2}' <<<"$info")"
        kill "$subscriber_pid" 2>/dev/null || true
        wait "$subscriber_pid" 2>/dev/null || true
        rm -f "$sample_file"
        [[ -z "$info" ]] || printf '%s\n' "$info"
        if [[ "$count" =~ ^[1-9][0-9]*$ ]]; then
            echo "电脑 Subscription 已发现，但 8 秒内未收到数据: $topic" >&2
        else
            echo "电脑未发现 Subscription endpoint，且 8 秒内未收到数据: $topic" >&2
        fi
        report_wireless_discovery_state
        return 1
    fi

    kill "$subscriber_pid" 2>/dev/null || true
    wait "$subscriber_pid" 2>/dev/null || true
    rm -f "$sample_file"
    echo "电脑临时探针已订阅并收到数据: $topic (检查前已有订阅者: $baseline_count)"
}

reset_ros_cli_graph() {
    ros2 daemon stop >/dev/null 2>&1 || true
}

check() {
    select_go2_network || return 1
    source_robot_env || return 1
    reset_ros_cli_graph
    printf 'ROS_DOMAIN_ID=%s, RMW_IMPLEMENTATION=%s, CycloneDDS 网卡=%s, 组播=开启\n' \
        "$ROS_DOMAIN_ID" "$RMW_IMPLEMENTATION" "$GO2_NETWORK_INTERFACE"
    if [[ -n "${GO2_DDS_PEER:-}" ]]; then
        printf 'CycloneDDS 静态发现 Peer=%s\n' "$GO2_DDS_PEER"
    fi
    echo '检查 Go2 L1 雷达和机身状态话题...'
    topic_info /utlidar/imu 'Publisher count' || return 1
    topic_info /utlidar/cloud 'Publisher count' || return 1
    topic_info /sportmodestate 'Publisher count' || return 1
    if pgrep -af '[t]ransform_everything' >/dev/null; then
        echo '检测到 SLAM 传感器转换节点；确认其原始 IMU 和点云订阅...'
        topic_info /utlidar/imu 'Subscription count' || return 1
        topic_info /utlidar/cloud 'Subscription count' || return 1
    else
        echo '当前未运行 SLAM 传感器转换节点；下面的订阅数是 check 临时探针，不代表 SLAM 订阅。'
    fi
    topic_sample /utlidar/imu || return 1
    topic_sample /utlidar/cloud || return 1
    topic_sample /sportmodestate || return 1
    echo '启动检查通过。此检查不会向 Go2 发送运动命令。'
}

check_motion_api() {
    local info count attempt=0
    while (( attempt < 10 )); do
        attempt=$((attempt + 1))
        info="$(ros2 topic info /api/sport/request 2>/dev/null || true)"
        count="$(awk -F': ' '$1 == "Subscription count" {print $2}' <<<"$info")"
        if [[ "$count" =~ ^[1-9][0-9]*$ ]]; then
            printf '%s\n' "$info"
            return 0
        fi
        (( attempt == 10 )) || sleep 1
    done
    if [[ ! "$count" =~ ^[1-9][0-9]*$ ]]; then
        if [[ "$GO2_SELECTED_INTERFACE" == "$GO2_WIFI_INTERFACE" ]]; then
            cat >&2 <<EOF
未发现 Go2 对 /api/sport/request 的订阅者。
请确认狗端 go2-wifi-relay.service 正在运行，且电脑已连接 Go2-Explore。
无线请求接收端不可见时，IMU 标定和实机运动控制链路不可用。
EOF
        else
            echo '未发现 Go2 对 /api/sport/request 的订阅者；运动命令链路不可用。' >&2
        fi
        [[ -z "$info" ]] || printf '%s\n' "$info" >&2
        return 1
    fi
}

build() {
    # shellcheck disable=SC1090
    source "$GO2_ROOT/unitree_setup.sh" || return 1
    if ! command -v colcon >/dev/null 2>&1; then
        echo '未找到 colcon；请先安装 python3-colcon-common-extensions。' >&2
        return 1
    fi
    cd "$GO2_ROOT"
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
}

calibrate_imu() {
    local answer calib_file backup
    select_go2_network || return 1
    source_robot_env || return 1
    reset_ros_cli_graph
    topic_info /utlidar/imu 'Publisher count' || return 1
    check_motion_api || return 1

    calib_file="$HOME/Desktop/imu_calib_data.yaml"
    mkdir -p "$HOME/Desktop"
    printf '\nIMU 标定会控制 Go2 原地旋转约 20 秒。请让狗站稳，在平整空地操作，清开周围人员和障碍物，并确认没有其他遥控/运动程序在运行。\n'
    read -r -p '确认环境安全后输入 CALIBRATE 开始: ' answer
    [[ "$answer" == CALIBRATE ]] || { echo '已取消。'; return 1; }

    if [[ -f "$calib_file" ]]; then
        backup="$calib_file.bak-$(date +%Y%m%d-%H%M%S)"
        cp -a "$calib_file" "$backup"
        echo "已备份旧标定: $backup"
    fi
    cd "$GO2_ROOT"
    python3 "$GO2_ROOT/tools/ros_process_group.py" --replace ros2 run calibrate_imu calibrate_imu || return $?
    if [[ -s "$calib_file" ]]; then
        echo "标定文件已生成: $calib_file"
    else
        echo "未发现标定文件: $calib_file" >&2
        return 1
    fi
}

slam() {
    local launch_pid="" link_lost launch_status interrupted=0 check_interval map_dir map_root
    select_go2_network || return 1
    source_robot_env || return 1
    reset_ros_cli_graph
    cd "$GO2_ROOT"
    check_interval=0.25
    [[ "$GO2_SELECTED_INTERFACE" == "$GO2_WIFI_INTERFACE" ]] && check_interval=1
    map_root="$GO2_ROOT/maps"
    mkdir -p "$map_root"

    trap 'interrupted=1; if [[ -n "$launch_pid" ]]; then kill -INT "$launch_pid" 2>/dev/null || true; fi' INT TERM
    while true; do
        map_dir="$(mktemp -d "$map_root/office_$(date +%Y%m%d-%H%M%S)-XXXXXX")"
        echo "本轮地图保存目录: $map_dir"
        echo "SLAM 监控已选链路 $GO2_SELECTED_INTERFACE；断线后将清理节点，并沿原链路恢复。"
        python3 "$GO2_ROOT/tools/ros_process_group.py" --replace ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=true "pcd_dir:=$map_dir" &
        launch_pid=$!
        link_lost=0
        while kill -0 "$launch_pid" 2>/dev/null; do
            if ! selected_link_up; then
                link_lost=1
                break
            fi
            (( interrupted )) && break
            sleep "$check_interval" || true
        done
        if (( ! interrupted )) && ! selected_link_up; then
            link_lost=1
        fi
        if (( interrupted )); then
            [[ -z "$launch_pid" ]] || kill -INT "$launch_pid" 2>/dev/null || true
            wait "$launch_pid" 2>/dev/null || true
            trap - INT TERM
            echo "地图文件（如已生成）保留在: $map_dir"
            return 130
        fi
        if (( link_lost )); then
            echo '检测到 Go2 链路断开，正在停止 SLAM，丢弃断线期间可能积压的传感器队列。'
            kill -INT "$launch_pid" 2>/dev/null || true
            wait "$launch_pid" 2>/dev/null || true
            launch_pid=""
            echo "断线前地图文件保留在: $map_dir"
            echo "等待 $GO2_SELECTED_INTERFACE 恢复；不会切换网络。"
            while ! selected_link_up ||
                { [[ "$GO2_SELECTED_INTERFACE" == "$GO2_WIRED_INTERFACE" ]] &&
                  ! ping -c 1 -W 1 -I "$GO2_WIRED_INTERFACE" 192.168.123.18 >/dev/null 2>&1; }; do
                (( interrupted )) && break
                sleep 1 || true
            done
            if (( interrupted )); then
                trap - INT TERM
                return 130
            fi
            sleep 2
            echo 'Go2 链路已恢复，重新创建 DDS 和 SLAM 节点。'
            continue
        fi
        if wait "$launch_pid"; then
            launch_status=0
        else
            launch_status=$?
        fi
        launch_pid=""
        trap - INT TERM
        echo "地图文件（如已生成）保留在: $map_dir"
        return "$launch_status"
    done
}

map_view() {
    local session="${1:-latest}" session_dir publisher_pid status=0
    session_dir="$(python3 "$GO2_ROOT/tools/map_results.py" resolve "$session")" || return 1
    python3 "$GO2_ROOT/tools/map_results.py" merge "$session_dir" || return 1
    # Keep map browsing isolated from the Go2 network and its ROS domain.
    # shellcheck disable=SC1091
    source /opt/ros/foxy/setup.bash || return 1
    export ROS_DOMAIN_ID=88
    export ROS_LOCALHOST_ONLY=1
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    unset CYCLONEDDS_URI
    python3 "$GO2_ROOT/tools/map_results.py" publish "$session_dir/map.pcd" &
    publisher_pid=$!
    trap 'kill "$publisher_pid" 2>/dev/null || true' INT TERM
    rviz2 -d "$GO2_ROOT/tools/map_view.rviz" || status=$?
    kill "$publisher_pid" 2>/dev/null || true
    wait "$publisher_pid" 2>/dev/null || true
    trap - INT TERM
    return "$status"
}

run_system() {
    local answer
    select_go2_network || return 1
    source_robot_env || return 1
    reset_ros_cli_graph
    printf '\n即将启动实机 local planner 和 RViz。请确认 Go2 站稳、场地安全且你准备好接管。\n'
    read -r -p '输入 RUN 启动，其他输入取消: ' answer
    [[ "$answer" == RUN ]] || { echo '已取消。'; return 1; }
    exec "$GO2_ROOT/system_real_robot.sh"
}

disconnect_go2_wifi() {
    require_network_tools || return 1
    if nmcli -t -f NAME connection show --active | grep -Fxq "$GO2_WIFI_SSID"; then
        nmcli connection down id "$GO2_WIFI_SSID"
        echo 'Go2 Wi-Fi 已断开；其他网络连接和有线网未改动。'
    else
        echo 'Go2 Wi-Fi 当前未连接。'
    fi
}

run_action() {
    case "$1" in
        connect) select_go2_network ;;
        network) show_network ;;
        check) check ;;
        diagnose) diagnostics ;;
        build) build ;;
        calibrate-imu) calibrate_imu ;;
        slam) slam ;;
        maps) python3 "$GO2_ROOT/tools/map_results.py" list ;;
        map-merge) python3 "$GO2_ROOT/tools/map_results.py" merge "${2:-latest}" ;;
        map-view) map_view "${2:-latest}" ;;
        run) run_system ;;
        disconnect) disconnect_go2_wifi ;;
        help|-h|--help) usage ;;
        *) usage >&2; return 2 ;;
    esac
}

if [[ $# -gt 0 ]]; then
    [[ $# -le 2 ]] || { usage >&2; exit 2; }
    run_action "$@"
    exit $?
fi

while true; do
    printf '\nGo2 实机工具\n'
    printf '1) 准备 Go2 链路（有线优先，无线回退）\n2) 启动检查\n3) 编译工作区\n4) IMU 标定（会让 Go2 原地转动）\n5) 独立 SLAM + RViz\n6) 启动 README 实机系统\n7) 网络诊断（不切换网络）\n8) 列出建图结果\n9) 查看最新地图\n0) 退出\n'
    read -r -p '选择: ' choice || exit 0
    case "$choice" in
        1) select_go2_network || echo 'Go2 网络未就绪。' ;;
        2) check || echo '启动检查未通过。' ;;
        3) build || echo '编译未完成。' ;;
        4) calibrate_imu || echo 'IMU 标定未完成。' ;;
        5) slam || echo 'SLAM 未启动。' ;;
        6) run_system || echo '实机系统未启动。' ;;
        7) diagnostics || echo '诊断未完成。' ;;
        8) python3 "$GO2_ROOT/tools/map_results.py" list ;;
        9) map_view || echo '地图查看器未能启动。' ;;
        0) exit 0 ;;
        *) echo '无效选项。' ;;
    esac
done
