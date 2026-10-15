# Go2 无线 DDS 转发需求

本文说明让外接 Ubuntu 电脑经 Go2-Explore 无线运行本仓库功能所需的 DDS 话题。当前中继只把传感器从狗端 `eth0` 转发到 `wlan0`；运动请求转发因回环故障暂停。

## 必需话题

| 方向 | ROS 2 话题 | 类型 | 用途 | 必需场景 |
|---|---|---|---|---|
| Go2 -> 电脑 | `/utlidar/imu` | `sensor_msgs/msg/Imu` | L1 雷达 IMU；传感器健康检查、SLAM、IMU 标定输入 | check、SLAM、标定、run |
| Go2 -> 电脑 | `/utlidar/cloud` | `sensor_msgs/msg/PointCloud2` | L1 雷达点云；健康检查、坐标变换和 SLAM | check、SLAM、run |
| Go2 -> 电脑 | `/sportmodestate` | `unitree_go/msg/SportModeState` | 机身运动状态健康检查 | check |
| 电脑 -> Go2 | `/api/sport/request` | `unitree_api/msg/Request` | Go2 Sport API 命令。标定节点发送原地旋转命令；路径跟随节点发送运动命令 | IMU 标定、实机 run |

上行 `/api/sport/request` 是让无线控制真正工作的关键。2026-10-15 发现旧版狗端中继会将狗自身的 StopMove 请求从 `wlan0` 转回 `eth0`，形成每秒大量请求的回环，导致原厂遥控器卡顿。当前已关闭该请求转发；无线传感器订阅和 SLAM 仍可用，但无线运动控制暂不可用。重新启用前必须按请求来源过滤，并验证没有回环。

## 可选话题

| 方向 | ROS 2 话题 | 类型 | 说明 |
|---|---|---|---|
| Go2 -> 电脑 | `/utlidar/cloud_deskewed` | `sensor_msgs/msg/PointCloud2` | 当前无线 relay 不转发；若其他工具需要，应先核算带宽 |
| Go2 -> 电脑 | `/lowstate` | `unitree_go/msg/LowState` | 当前无线 relay 不转发；低层状态诊断和 SLAM 不依赖 |

## 不需要跨机器转发

`/utlidar/transformed_imu`、`/utlidar/transformed_cloud`、`/state_estimation`、`/registered_scan`、`/terrain_map`、`/path` 和 `/cmd_vel` 都由电脑上的传感器变换、SLAM 或规划节点产生并消费。这些是电脑本机 ROS 图中的处理结果，不是狗端原始话题，不要为了无线接入额外做狗端转发。

`/cmd_vel` 在 IMU 标定和路径跟随代码中用于本地调试/状态输出；Go2 实际收到的运动命令是 `/api/sport/request`，不能把二者混为一谈。

## DDS 和转发要求

- Go2 与电脑两端使用 Domain 0；电脑端 CycloneDDS 在无线模式只绑定 `wlp4s0`，静态发现 Peer 为 `192.168.76.1`，`ROS_LOCALHOST_ONLY=0`。
- 转发保持 ROS 话题名和消息类型不变，并使用与现有 ROS 2 DDS QoS 兼容的配置。传感器数据方向为 Go2 `eth0` -> Go2 `wlan0` -> 电脑；运动请求方向为电脑 `wlp4s0` -> Go2 `wlan0` -> Go2 `eth0` 上的 Sport API 消费者。
- 请求转发只走电脑到狗端，传感器只走狗端到电脑；不要把转发器接收到的数据重新发回来源接口，以免形成回环或重复流量。
- 无线传感器仅转发 IMU、原始点云和机身状态；relay 的无线发布使用 Reliable 和有界历史队列。Go2 AP 为 2.4 GHz、20 MHz，额外转发 deskewed 点云会显著挤占 SLAM 余量。
- 默认模式仍优先使用电脑 `enp2s0`，其上不依赖无线中继。无线测试可以保持网线连接，并显式强制 DDS 只用 `wlp4s0`；确认电脑关联 Go2-Explore 且地址为 `192.168.76.2/24`。

## 验收步骤

在仓库目录运行以下命令。`GO2_FORCE_WIFI=1` 会选择 Wi-Fi，即使 Go2 网线仍连接；若电脑尚未连接 Go2-Explore，它会使用已保存的 NetworkManager 配置连接热点：

```bash
GO2_FORCE_WIFI=1 ./go2.sh check
```

检查输出必须显示 `Go2 链路: Wi-Fi (wlp4s0, 强制选择)` 和 `CycloneDDS 网卡=wlp4s0`。IMU、点云、运动状态话题的 `Publisher count` 应大于 0，之后还应分别看到 `收到实际数据样本`。仅看见 `Publisher count: 1` 只证明发现了发布端；`Subscription count: 0` 表示当时没有订阅节点，不证明样本已收到。已验证在网线保持连接时，三个话题均可经 `wlp4s0` 发现。ARP 抓包也确认电脑 `.2` 与 Go2 `.1` 双向请求/回复正常；邻居表带 `lladdr` 时，即使状态为 `STALE`、`DELAY` 或 `PROBE`，也不应误报为 ARP 未解析。

当前无线模式不应期待中继提供 `/api/sport/request` 的订阅端；无线 IMU 标定与运动控制在请求转发修复前不可用。

脚本对应关系：`./go2.sh check` 默认有线优先；`GO2_FORCE_WIFI=1 ./go2.sh check` 强制检查 Wi-Fi；`GO2_FORCE_WIFI=1 ./go2.sh slam` 可在保留网线时通过 Wi-Fi 启动 SLAM。`slam` 和 `run` 会准备网络与 ROS 环境，但不预检话题；`./go2.sh calibrate-imu` 仍要求 `/api/sport/request` 的狗端订阅者可见。若要让后续操作也强制走 Wi-Fi，可在启动命令前设置 `export GO2_FORCE_WIFI=1`。
