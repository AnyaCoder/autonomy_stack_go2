# Go2 实机操作手册（外接 Ubuntu）

本手册涵盖网络连接、启动检查、编译、L1 IMU 标定和独立 SLAM。所有操作统一使用 `./go2.sh`，命令默认在本仓库目录 `/home/anyacoder/autonomy_stack_go2` 执行。

## 1. 开机与连接

1. 打开 Go2，等待它完成启动并站稳。使用 EDU 版 Go2；遥控器或 App 不要同时发送运动命令。
2. 有线调试时，把 Ubuntu 的 `enp2s0` 接到 Go2 后部网口；电脑地址为 `192.168.123.100/24`，Go2 默认是 `192.168.123.18/24`。无线调试可直接使用 Go2-Explore 热点，不要求插网线。
3. 登录 Ubuntu 后选择 ROS 2 Foxy 环境，不要选 ROS 1。检查系统时间正常；Go2 机载电脑若时间回到 1970，按 README 连接 Wi-Fi 校时后再使用。
4. 打开终端并进入仓库：

   ```bash
   cd ~/autonomy_stack_go2
   ```

   新克隆或尚未编译过的工作区，先按下一节运行一次 `./go2.sh build`。使用工具前准备当前可用的 Go2 链路并做检查：

   ```bash
   ./go2.sh connect
   ./go2.sh check
   ```

   `connect` 使用 NetworkManager 已保存的 Go2 热点配置，不询问密码。默认有线优先；有线不可达时连接热点。该选择在每次命令启动时进行，不是运行中的自动热切换。

## 2. 编译

首次使用或修改代码后运行：

```bash
./go2.sh build
```

等价于 README 的 Release 构建命令：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

如果缺少依赖，按 README 的 Real-robot Setup 安装 ROS 2 Foxy 依赖、`python3-colcon-common-extensions`、`transforms3d` 和 `pyyaml`，并先完成 Unitree ROS 2 / Cyclone DDS 设置。当前 `unitree_setup.sh` 使用 `~/cyclonedds-0.10.2/install`；可用 `GO2_CYCLONEDDS_PREFIX` 指定其他安装位置。

## 3. L1 IMU 标定

README 建议每台 Go2 标定一次。标定会通过 Go2 Sport API 控制机身，并在后半段以约 `1.396 rad/s` 原地旋转约 20 秒；需要平整、宽敞、无人的场地。让 Go2 先站稳，确认周围没有障碍物，也没有其他导航或遥控程序发送运动命令。

运行：

```bash
./go2.sh calibrate-imu
```

脚本先检查雷达 IMU 话题和所选链路的 Go2 运动 API 订阅端，然后要求输入 `CALIBRATE` 才继续。若桌面已有 `imu_calib_data.yaml`，脚本会先按时间戳备份。无线标定需强制选择 Go2 Wi-Fi：

```bash
GO2_NETWORK_MODE=wifi ./go2.sh calibrate-imu
```

无线时标定节点把 Sport 请求 remap 到 `/go2/wifi/sport/request`，由狗端 `go2-wifi-relay.service` 单向转发至原生 `/api/sport/request`。脚本会在要求确认前检查无线专用话题上是否发现狗端 relay；找不到订阅者就停止。默认 `auto` 仍是有线优先。标定节点约运行 37 秒，完成后写入：

```text
~/Desktop/imu_calib_data.yaml
```

标定会让 Go2 调整姿态并原地旋转约 20 秒。只有在狗已开机、站稳且场地安全时才输入 `CALIBRATE`；如需中止，使用遥控器/App 的安全停止方式并确保狗已停稳。

系统启动时由 `transform_sensors` 读取该文件。完成后再次运行 `./go2.sh check` 确认传感器话题仍正常。

## 4. 独立 SLAM

独立 SLAM 只启动 L1 数据转换、Point-LIO 和其 RViz 配置，不启动 local planner、FAR Planner、`joy_node` 或自主导航控制。先关闭其他终端里正在运行的 Point-LIO / SLAM，避免重复订阅和重复 RViz；快捷脚本会检测常见的 Point-LIO 进程。

```bash
./go2.sh slam
```

SLAM 实际启动内容相当于：

```bash
source /opt/ros/foxy/setup.bash
source ./unitree_setup.sh
source ./install/setup.bash
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=true
```

正常运行时可在 RViz 查看 L1 点云和里程计。启动初期若数据延迟或地图尚未稳定，README 提醒可能需要等待 10–20 秒。启动时 `level_map` 会先发布单位 `map -> camera_init` 变换，在狗静止且地面可拟合时测量前几帧注册点云并更新为找平后的变换；RViz 使用 `camera_init` 固定坐标系。点云显示使用 Best Effort QoS 和短队列，避免无线环境堆积旧帧。若画面为空，运行 `./go2.sh check`；SLAM 运行中会依次验证原始、变换后和 Point-LIO 注册点云，首个失败阶段就是排查位置。`./go2.sh slam` 会按每 600 帧分块保存 Point-LIO 地图，目录为 `maps/office_时间戳-随机号/`；正常 Ctrl+C 退出时，剩余点云会写入 `scans.pcd`，此前的 `scans_*.pcd` 分块也会保留。链路断开导致重启时，每轮数据写入不同目录。`./go2.sh maps` 列出批次和保存状态；`./go2.sh map-merge latest` 将最新批次的分块流式合并为 `map.pcd`；`./go2.sh map-view latest` 在独立本机 ROS Domain 中打开 RViz 查看整张地图，可把 `latest` 换成列表里的批次名。默认查看和合并最新批次，不会切换网络或连接 Go2。RViz 的 Save Config 保存 `.rviz` 显示配置，不保存点云地图。脚本监控启动时选中的 Go2 链路：有线断开或无线热点失联时会停止整组 SLAM 节点，待原链路恢复后重建 DDS/SLAM；运行中不会自动切换到另一张网卡。

当前工作区的 `mapping_utlidar.launch` 会加载 `transform_sensors/config/l1_extrinsics.yaml` 中的 L1 旋转参数。转换节点将原始加速度保留在 `/utlidar/transformed_raw_imu`，而供 Point-LIO 使用的 `/utlidar/transformed_imu` 暂时只提供已校正角速度，加速度置零；加速度外参尚未验证。`mapping.imu_en` 仍为 `true`，用于接收 IMU 时间序列。同一录制数据回放中，`use_imu_as_input: true` 的运动段姿态 RMS 为 22.1°、最终误差 48.9°，`false` 为 13.4°、最终误差 8.9°。平移仍沿用仓库原值，IMU-LiDAR 时间差保持 `0.0`，因为非零候选未通过独立回放验证。狗转圈时的姿态稳定性仍需实机复测。

## 常用命令

| 任务 | 命令 |
|---|---|
| 打开交互菜单 | `./go2.sh` |
| 准备 Go2 链路（有线优先，无线回退） | `./go2.sh connect` |
| 查看当前网络 | `./go2.sh network` |
| 检查通信 | `./go2.sh check` |
| 编译 | `./go2.sh build` |
| 标定 L1 IMU | `./go2.sh calibrate-imu` |
| 独立 SLAM | `./go2.sh slam` |
| 列出建图批次 | `./go2.sh maps` |
| 合并最新批次 | `./go2.sh map-merge` |
| 查看最新地图 | `./go2.sh map-view` |
| 启动 README 实机系统 | `./go2.sh run` |
| 网络与 ROS 诊断 | `./go2.sh diagnose` |
| 查看帮助 | `./go2.sh --help` |

`./go2.sh slam` 和 `./go2.sh run` 不会预检话题或数据样本；它们会准备网络与 ROS 环境后直接启动。`run` 会清理旧进程并要求输入 `RUN`；`run far` 选择 README 的 FAR 实机启动文件。

## 5. 无线连接 Go2

Go2 当前的 `Go2-Explore` 是 WPA2 热点，地址为 `192.168.76.1/24`，不提供互联网。电脑的 AIC USB 网卡默认为 `wlx688fc9003c5d`，NetworkManager 连接名是 `Go2-Explore-AIC`，热点 SSID 是 `Go2-Explore`；配置使用 `192.168.76.2/24`，不设默认网关。AIC 是独立网卡，连接 Go2 不会替换内置 Wi-Fi 的校园网默认路由。

```bash
GO2_NETWORK_MODE=wifi ./go2.sh connect
GO2_NETWORK_MODE=wifi ./go2.sh check
```

网络选择由 `GO2_NETWORK_MODE` 管理：`auto`（默认）有线优先、无线回退；`wired` 只使用有线；`wifi` 即使网线可达也强制使用 Wi-Fi。旧变量 `GO2_FORCE_WIFI=1` 仍兼容，等同 `GO2_NETWORK_MODE=wifi`。NetworkManager 需预先保存热点密码；脚本只配置静态本地地址，不读取或输出密码。热点连接不新增默认路由。无线模式可检查传感器、运行 SLAM、实机系统和 IMU 标定；无线标定请求经专用 relay 话题转发。

需要更换网卡或 NetworkManager 连接名时，可覆盖以下默认值：

| 变量 | 默认值 | 用途 |
|---|---|---|
| `GO2_WIRED_INTERFACE` | `enp2s0` | 电脑有线网卡 |
| `GO2_WIRED_HOST` | `192.168.123.18` | Go2 有线地址 |
| `GO2_WIFI_INTERFACE` | `wlx688fc9003c5d` | AIC USB Wi-Fi 网卡 |
| `GO2_WIFI_CONNECTION` | `Go2-Explore-AIC` | NetworkManager 连接名 |
| `GO2_WIFI_SSID` | `Go2-Explore` | Go2 热点名称 |
| `GO2_WIFI_HOST` | `192.168.76.1` | Go2 无线地址及 DDS Peer |
| `GO2_WIFI_ADDRESS` | `192.168.76.2/24` | 电脑无线地址 |
| `GO2_SPORT_REQUEST_TOPIC` | `/api/sport/request` | 有线 Sport 请求话题 |
| `GO2_WIFI_SPORT_REQUEST_TOPIC` | `/go2/wifi/sport/request` | relay 的无线专用 Sport 请求话题 |

例如使用内置网卡时，同时指定其 NetworkManager 连接名：`GO2_NETWORK_MODE=wifi GO2_WIFI_INTERFACE=wlp4s0 GO2_WIFI_CONNECTION=Go2-Explore ./go2.sh check`。

热点可达只证明 IP 网络通，不代表 ROS 2 DDS 已经通。无线模式下电脑使用 Domain 0、`ROS_LOCALHOST_ONLY=0`、组播及 CycloneDDS 静态发现 Peer `192.168.76.1`。Go2 relay 将传感器从 `eth0` 转发到 `wlan0`，包括供遥控器接管检测的 `/lowstate`；无线导航命令从专用 `/go2/wifi/sport/request` 单向转入 eth0 的 `/api/sport/request`，避免原生 StopMove 回环。`./go2.sh check` 实际订阅 IMU、点云和运动状态；发现端点和狗端写入计数均不能代替电脑端实际接收验证。详细话题见 [Go2 无线 DDS 转发需求](Go2无线DDS转发需求.md)。

若无线话题发现失败，保持电脑关联 Go2-Explore 后，强制走 Wi-Fi 运行检查并保存完整终端输出：

```bash
GO2_NETWORK_MODE=wifi ./go2.sh check
```

`GO2_NETWORK_MODE=wifi ./go2.sh check` 会在热点尚未连接时激活已保存配置，并检查电脑是否实际收到所需话题；它不会发送运动命令。强制有线检查使用 `GO2_NETWORK_MODE=wired ./go2.sh check`；不指定模式时使用有线优先的 `auto`。只读诊断可运行 `GO2_NETWORK_MODE=wifi ./go2.sh diagnose`，该命令不会主动切换连接。邻居表中有 Go2 的 `lladdr` 时，`PROBE` 是可达性探测状态，不代表 ARP 地址未解析。

检查通过后，准备启动 README 的实机系统时运行：

```bash
./go2.sh run
```

该命令自动选链路、准备 ROS 环境并要求输入 `RUN` 才启动实机系统，不预检话题。有线直接发布运动 API；无线自动重映射到 relay 专用命令输入。`run` 的 RViz 左侧展开 `RegScan`，修改 `Decay Time` 可调整点云显示的保留秒数。同一窗口显示 Go2 前置相机、D435i 彩色和深度图；对应狗端相机服务需运行。`./go2.sh disconnect` 仅断开 Go2 热点，不删除已保存连接。

`./go2.sh run`（或 `run local`）是近距离 Waypoint 局部导航。需要 README 图片里的红色障碍轮廓、青色可视图和全局路径时使用 `./go2.sh run far`；它启动原有 FAR 实机 launch，配套 RViz 的 `Goalpoint` 用于设置全局目标。红线是障碍多边形轮廓，并非保证可通行的区域边界；粗蓝线是尝试经过未知区域的全局路径，已知自由空间内的路径为绿色。全局路径只有设置目标、规划成功后才有，未设目标时不应期待蓝线出现。

### RViz 路径点导航

在 `run` 中，等 SLAM 地图和车体坐标稳定后，用 RViz 的 `Waypoint` 工具点击车附近、已观察到的可通行位置。点击会切到自主模式；局部规划器用当前点云避障，跟随器先对准局部路径，再以最高 0.35 m/s 前进、最高 0.5 rad/s 转向。约 0.4 m 内判定到达；路径/里程计过期或 20 秒内未向目标推进至少 0.1 m 时最多发送一次 `StopMove`，之后保持静默。停止后重新点击目标才能开始新的一轮。

实机跟随器还要收到 `/lowstate` 中的原厂遥控器状态才会发送运动请求；遥控器摇杆或按键一动，即取消当前目标并立即停止发布请求，接管时不会发 `StopMove` 与遥控器争抢。遥控器松开后，局部 Waypoint 需重新点击目标才能恢复。无线 relay 已转发 `/lowstate` 并接收专用运动命令；本次未切换电脑 Wi-Fi，仍需电脑无线端到端验证。

`Waypoint` 是近距离局部导航，不能保证绕过整个办公室的死角；跨区域目标使用 FAR 全局规划。第一次上狗时先在空地点击车正前方约 1 m 的目标，并由操作者握持原厂遥控器准备接管。当前代码已做离线 ROS 消息测试，尚未在重启后的实机验证运动、避障和到达效果。

### Unity 仿真验证

在本机运行 `./system_simulation.sh`，等待 Unity 场景、RViz 和点云出现，再用 RViz 的 `Waypoint` 工具点击车附近目标。脚本默认使用独立的 ROS Domain 94，并将 CycloneDDS 绑定在本机回环网卡，不连接或控制实机 Go2；需要并行运行另一个仿真实例时可设置 `GO2_SIM_DOMAIN_ID`。已在安装的 Unity 场景中验证前方 1 米目标到达并停稳。退出脚本时会关闭它启动的 Unity 场景。

### ROS Domain 统一约定

| Domain | 用途 | 网络范围 |
|---|---|---|
| 0 | 实机有线/无线、SLAM、FAR、原厂传感器与无线 relay | 实机链路 |
| 76 | 狗端相机和辅助传感器服务，保留原有隔离 | 狗端服务；电脑使用 HTTP 相机预览 |
| 94 | 两个 Unity 启动脚本 | 本机回环 |
| 91 | 地图查看、离线标定回放、自动化 ROS/DDS 测试 | 仅本机 |

历史默认值 88、92、179、180 已停用，临时测试 119 不再使用。狗端相机和辅助服务尝试迁入 0 后触发旧版 Fast DDS 的 `bad_alloc`，已恢复原配置与 Domain 76，HTTP 预览地址不变。`ROS_DOMAIN_ID` 由启动脚本管理，日常不需要手动 source 后再设置。

Domain 91 的地图查看、离线回放和测试应顺序运行，不要同时启动。baseline 和 calibrated 回放也按顺序执行；结果文件名仍分别保存。Unity 与实机可以同时运行，禁止将仿真或离线回放的域改为 0。

### FAR 障碍轮廓与窄通道

轮廓提取先用 3×3 邻域统计点数过滤噪声，但只把有实际点的栅格视为障碍，再在细化图像上加 `c_detector/contour_padding_m` 边距。当前配置为 0.05 m，按 0.1 m 栅格、3 倍放大取整为约 0.067 m；不再把整圈邻域直接填为障碍，也不再用机身净空决定轮廓图像模糊核。该修改适用于共用轮廓提取器的实机与仿真。

FAR `robot_dim=0.6`、导航净空计算、局部规划器 `vehicleWidth=0.7` 和路径碰撞检查保持原值；红线缩小不等于保证狗能通过。固定点云回归中，1 m 双墙走廊的轮廓图空白宽度由约 0.53 m 增为 0.77 m；这不包括真实感知误差或机身净空，也不是实机通行验证。稀疏墙面连通、孤立噪声过滤和有足够回波的小障碍均有图像级测试。

独立回归测试（不使用 ROS 或连接狗）：

```bash
g++ -std=c++14 tests/test_far_contour_image.cpp -I src/route_planner/far_planner/include $(pkg-config --cflags --libs opencv4) -o /tmp/test_far_contour_image
/tmp/test_far_contour_image
```

导航实时性设置：`run` 和 `run far` 关闭 Point-LIO 自动 PCD 落盘，避免同步写文件影响处理；独立 `slam` 仍自动保存地图。FAR RViz 的 `RegScan` 默认保留 20 秒点云，订阅深度和 TF 等待队列均为 2，使用 Best Effort 接收以降低显示端积压。这些设置不改变 SLAM 的匹配、外参或时间参数，也不改变规划器接收点云的策略。
