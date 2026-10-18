# Go2 无线 DDS 转发需求

本文说明外接 Ubuntu 电脑经 Go2-Explore 无线运行本仓库功能所需的 DDS 话题。传感器由狗端 `eth0` 转发到 `wlan0`；电脑无线运动命令使用独立话题，再单向转入狗端 Sport API。

## 必需话题

| 方向 | ROS 2 话题 | 类型 | 用途 | 必需场景 |
|---|---|---|---|---|
| Go2 -> 电脑 | `/utlidar/imu` | `sensor_msgs/msg/Imu` | L1 雷达 IMU；传感器健康检查、SLAM、IMU 标定输入 | check、SLAM、标定、run |
| Go2 -> 电脑 | `/utlidar/cloud` | `sensor_msgs/msg/PointCloud2` | L1 雷达点云；健康检查、坐标变换和 SLAM | check、SLAM、run |
| Go2 -> 电脑 | `/sportmodestate` | `unitree_go/msg/SportModeState` | 机身运动状态健康检查 | check |
| Go2 -> 电脑 | `/lowstate` | `unitree_go/msg/LowState` | 原厂遥控器状态，供路径跟随器检测接管；已加入转发 | 实机 run |
| 电脑 -> Go2 | `/go2/wifi/sport/request` | `unitree_api/msg/Request` | 无线导航和 IMU 标定专用输入，relay 接收后经 eth0 发布为 `/api/sport/request` | 无线实机 run、无线 IMU 标定 |
| 电脑 -> Go2 | `/api/sport/request` | `unitree_api/msg/Request` | 有线直接使用的 Go2 Sport API 命令；不作为无线 relay 输入 | 有线 IMU 标定、实机 run |

2026-10-15 的旧中继将狗自身的 StopMove 请求从 `wlan0` 转回 `eth0`，形成回环。现在 relay 仅订阅 `/go2/wifi/sport/request`，不订阅原生 `/api/sport/request`。输入和输出话题不同，狗自身的运动请求不会再次进入转发。`go2.sh run` 和 `go2.sh run far` 在无线模式自动重映射路径跟随器的输出；有线保持原生话题。中继不主动产生 Move 或 StopMove；进程间队列积压超过 0.3 秒的请求被丢弃。

## 可选话题

| 方向 | ROS 2 话题 | 类型 | 说明 |
|---|---|---|---|
| Go2 -> 电脑 | `/utlidar/cloud_deskewed` | `sensor_msgs/msg/PointCloud2` | 当前无线 relay 不转发；若其他工具需要，应先核算带宽 |

## 不需要跨机器转发

`/utlidar/transformed_imu`、`/utlidar/transformed_cloud`、`/state_estimation`、`/registered_scan`、`/terrain_map`、`/path` 和 `/cmd_vel` 都由电脑上的传感器变换、SLAM 或规划节点产生并消费。这些是电脑本机 ROS 图中的处理结果，不是狗端原始话题，不要为了无线接入额外做狗端转发。

`/cmd_vel` 在 IMU 标定和路径跟随代码中用于本地调试/状态输出；Go2 实际收到的运动命令是 `/api/sport/request`，不能把二者混为一谈。

## DDS 和转发要求

- Go2 与电脑两端使用 Domain 0；电脑端 CycloneDDS 在无线模式只绑定 `$GO2_WIFI_INTERFACE`（当前默认 `wlx688fc9003c5d`），静态发现 Peer 为 `192.168.76.1`，`ROS_LOCALHOST_ONLY=0`。
- 传感器保持话题名和类型不变。运动请求类型保持 `unitree_api/msg/Request`，输入 `/go2/wifi/sport/request`，输出 `/api/sport/request`。传感器方向为 Go2 `eth0` -> Go2 `wlan0` -> 电脑；运动请求方向相反。
- 请求转发只走电脑到狗端，传感器只走狗端到电脑；不要把转发器接收到的数据重新发回来源接口，以免形成回环或重复流量。
- 无线传感器转发 IMU、原始点云、机身状态和 `/lowstate`；relay 的无线发布使用 Reliable 和有界历史队列。Go2 AP 为 2.4 GHz、20 MHz，额外转发 deskewed 点云会显著挤占 SLAM 余量。
- 默认 `GO2_NETWORK_MODE=auto` 优先使用电脑 `enp2s0`，其上不依赖无线中继。无线测试可以保持网线连接，并用 `GO2_NETWORK_MODE=wifi` 强制 DDS 只用 `$GO2_WIFI_INTERFACE`；确认电脑关联 Go2-Explore 且地址为 `192.168.76.2/24`。

## 验收步骤

在仓库目录运行以下命令。`GO2_NETWORK_MODE=wifi` 会选择 Wi-Fi，即使 Go2 网线仍连接；若电脑尚未连接 Go2-Explore，它会使用已保存的 NetworkManager 配置连接热点：

```bash
GO2_NETWORK_MODE=wifi ./go2.sh check
```

检查输出应显示 `Go2 链路: Wi-Fi (wlx688fc9003c5d, Go2-Explore)` 和 `CycloneDDS 网卡=wlx688fc9003c5d`。IMU、点云、运动状态话题的 `Publisher count` 应大于 0，之后还应分别看到 `电脑临时探针已订阅并收到数据`。仅看见 `Publisher count: 1` 只证明发现了发布端；`Subscription count: 0` 表示当时没有订阅节点，不证明样本已收到。已验证 AIC 网卡在网线保持连接时，三个话题均可经 Wi-Fi 收到样本。ARP 邻居状态为 `PROBE` 时，只要仍有 `lladdr` 就不代表地址未解析。

无线模式应发现 `/go2/wifi/sport/request` 的狗端订阅者。`/api/sport/request` 的真实消费者在内部 eth0 上，不要求从电脑 Wi-Fi 看见。`./go2.sh calibrate-imu` 会按所选链路 remap Sport 请求；无线标定使用 `GO2_NETWORK_MODE=wifi ./go2.sh calibrate-imu`，启动前会检查 relay 订阅端。

脚本对应关系：`./go2.sh check` 默认有线优先；`GO2_NETWORK_MODE=wifi ./go2.sh check` 强制检查 Wi-Fi；`GO2_NETWORK_MODE=wired ./go2.sh check` 强制检查有线；相同模式可用于 `slam`、`run` 和 `calibrate-imu`。旧变量 `GO2_FORCE_WIFI=1` 仍兼容 Wi-Fi 模式。Wi-Fi 网卡、NetworkManager profile、本机地址和无线 Sport 入口可分别通过 `GO2_WIFI_INTERFACE`、`GO2_WIFI_CONNECTION`、`GO2_WIFI_ADDRESS`、`GO2_WIFI_SPORT_REQUEST_TOPIC` 覆盖。

## 2026-10-16 部署验证

`tools/native_wifi_relay.py` 已同步部署到狗端 `~/go2/relay/native_wifi_relay.py`，并重启用户服务。狗端仅绑定 wlan0 的 8 秒订阅测试收到 `/lowstate` 730 条；这不等于电脑 Wi-Fi 端到端验证，电脑本次保持有线。原版本备份在 `~/go2/archive/native_wifi_relay.pre-lowstate-20261016.py`。

有线 `run` 的 12 秒检查收到 `/lowstate` 3258 条，SLAM 位姿、注册点云、地形图和规划路径均有实际数据；`pathFollower` 已订阅 `/lowstate`。采样期间遥控器字节全为零，尚未通过拨动原厂手柄验证接管。运动 API 有狗端订阅者，采样期间未收到运动请求；未发送导航目标。
