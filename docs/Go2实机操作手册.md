# Go2 实机操作手册（外接 Ubuntu）

本手册涵盖网络连接、启动检查、编译、L1 IMU 标定和独立 SLAM。所有操作统一使用 `./go2.sh`，命令默认在本仓库目录 `/home/anyacoder/autonomy_stack_go2` 执行。

## 1. 开机与连接

1. 打开 Go2，等待它完成启动并站稳。使用 EDU 版 Go2；遥控器或 App 不要同时发送运动命令。
2. 用网线把 Ubuntu 机器的有线网卡接到 Go2 后部网口。README 推荐 Ubuntu 有线网卡 `enp2s0` 配为 `192.168.123.100/24`；Go2 机载电脑默认是 `192.168.123.18/24`。
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

   `connect` 使用 NetworkManager 已保存的 Go2-Explore 配置，不询问密码。有线 Go2 可达时只使用有线；否则连接热点并只使用 Wi-Fi。该选择在每次命令启动时进行，不是运行中的自动热切换。Go2 热点不提供互联网，连接时会断开同一张 Wi-Fi 网卡上的校园网。

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

脚本会先检查雷达 IMU 话题和 Go2 运动 API 订阅端，然后要求输入 `CALIBRATE` 才继续。若桌面已有 `imu_calib_data.yaml`，脚本会先按时间戳备份。标定节点约运行 37 秒，完成后写入：

```text
~/Desktop/imu_calib_data.yaml
```

系统启动时由 `transform_sensors` 读取该文件。标定期间不要关闭终端或按 Ctrl+C；如需中止，使用遥控器/App 的安全停止方式并确保狗已停稳。完成后再次运行 `./go2.sh check` 确认传感器话题仍正常。

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

正常运行时可在 RViz 查看 L1 点云和里程计。启动初期若数据延迟或地图尚未稳定，README 提醒可能需要等待 10–20 秒。启动时 `level_map` 会先发布单位 `map -> camera_init` 变换，在狗静止且地面可拟合时测量前几帧注册点云并更新为找平后的变换；RViz 网格使用 `map`，原始 Point-LIO 仍使用 `camera_init`。找平失败时 20 秒后保留单位变换并打印告警，应检查地面和点云质量。`./go2.sh slam` 会按每 600 帧分块保存 Point-LIO 地图，目录为 `maps/office_时间戳-随机号/`；正常 Ctrl+C 退出时，剩余点云会写入 `scans.pcd`，此前的 `scans_*.pcd` 分块也会保留。链路断开导致重启时，每轮数据写入不同目录。`./go2.sh maps` 列出批次和保存状态；`./go2.sh map-merge latest` 将最新批次的分块流式合并为 `map.pcd`；`./go2.sh map-view latest` 在独立本机 ROS Domain 中打开 RViz 查看整张地图，可把 `latest` 换成列表里的批次名。默认查看和合并最新批次，不会切换网络或连接 Go2。RViz 的 Save Config 保存 `.rviz` 显示配置，不保存点云地图。脚本监控启动时选中的 Go2 链路：有线断开或无线热点失联时会停止整组 SLAM 节点，待原链路恢复后重建 DDS/SLAM；运行中不会自动切换到另一张网卡。无线仍关联热点但 DDS 数据中断时，需用 `./go2.sh check` 检查实际样本。

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

`./go2.sh slam` 和 `./go2.sh run` 不会预检话题或数据样本；它们会准备网络与 ROS 环境后直接启动。`run` 仍会检查重复进程并要求输入 `RUN`，再启动 `system_real_robot.sh`。README 中的 `system_real_robot_with_route_planner.sh` 仍需在确认场地和操作者准备好后单独运行。

## 5. 无线连接 Go2

Go2 当前的 `Go2-Explore` 是 WPA2 热点，地址为 `192.168.76.1/24`，不提供互联网。脚本把电脑的 Go2-Explore 配置为 `192.168.76.2/24`，不设默认网关；该地址位于 Go2 热点 DHCP 池之外。使用统一脚本：

```bash
GO2_FORCE_WIFI=1 ./go2.sh connect
GO2_FORCE_WIFI=1 ./go2.sh check
```

NetworkManager 已保存 Go2-Explore 配置和密码。`go2.sh` 会把该配置设为静态本地地址，不会询问或输出密码；如果配置被删除，脚本会提示需要在 Ubuntu 网络设置中重新保存。单张 Wi-Fi 网卡连接 Go2 热点时会断开校园 Wi-Fi，热点不提供互联网。热点连接不会新增默认路由。默认情况下有线 Go2 可达时优先绑定有线网卡；设置 `GO2_FORCE_WIFI=1` 可在网线保持连接时强制让 Cyclone DDS 只绑定 Wi-Fi 网卡，适合验证无线话题链路。

热点可达只证明 IP 网络通，不代表 ROS 2 DDS 已经通。无线模式下电脑使用 Domain 0、`ROS_LOCALHOST_ONLY=0`、组播及 CycloneDDS 静态发现 Peer `192.168.76.1`。Go2 上的无线 relay 将传感器话题从 `eth0` 转发到 `wlan0`；运动请求转发因 StopMove 回环故障暂停，无线模式暂不能控制狗运动。`./go2.sh check` 会检查 Domain 0 的 L1 IMU、点云和机身状态发布者，并用 sensor-data QoS 实际订阅三者、确认收到样本。单独运行 `ros2 topic info` 看到 `Publisher count: 1` 只证明发现了发布端；`Subscription count: 0` 表示当时没有订阅节点，不能证明样本已收到。无线链路应以电脑实际收到样本为准，不能用 Go2 本机 relay 写入计数代替端到端验证。详细话题和检查方式见 [Go2 无线 DDS 转发需求](Go2无线DDS转发需求.md)。

若无线话题发现失败，保持电脑关联 Go2-Explore 后，强制走 Wi-Fi 运行检查并保存完整终端输出：

```bash
GO2_FORCE_WIFI=1 ./go2.sh check
```

`GO2_FORCE_WIFI=1 ./go2.sh check` 会在 Go2 Wi-Fi 尚未连接时激活已保存的热点配置，并检查电脑是否实际发现所需话题；它不会发送运动命令。只读网络和 ROS 诊断仍可运行 `./go2.sh diagnose`，若电脑已经连接 Go2-Explore，可用 `GO2_FORCE_WIFI=1 ./go2.sh diagnose` 明确选择 Wi-Fi。邻居表中有 Go2 的 `lladdr` 时，`PROBE` 是可达性探测状态，不代表 ARP 地址未解析。

检查通过后，准备启动 README 的实机系统时运行：

```bash
./go2.sh run
```

该命令会自动选链路、准备 ROS 环境并要求输入 `RUN` 才启动实机系统。它不预检传感器话题或 `/api/sport/request` 订阅者；当前只能在有线模式下进行路径点运动控制。`run` 的 RViz 左侧展开 `RegScan`，修改 `Decay Time` 可调整注册点云在屏幕上的保留秒数；这只影响显示，不改变 SLAM 数据或传输延迟。同一窗口显示 Go2 前置相机、D435i 彩色图和深度图；狗端 `go2-front-camera.service` 与 `go2-realsense-preview.service` 需在运行。需要断开 Go2 热点时运行 `./go2.sh disconnect`；NetworkManager 中保存的连接不会被删除。
