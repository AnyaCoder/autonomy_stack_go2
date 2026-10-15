# Go2 调试与部署记录

这段时间主要做了三件事：把 `autonomy_stack_go2` 放到 Go2 上跑起来，打通 L1 雷达数据和 IMU 校准，再把 SLAM 放到实机上验证。无线部分也整理成了 `~/go2-wireless`，用中继把狗端和电脑端的 ROS 2 数据接起来。

## 先把项目跑起来

Go2 用 EDU 版，主机上跑 ROS 2 Foxy，工作区在 `~/autonomy_stack_go2`。电脑通过以太网或 Go2 热点连接机器狗；环境初始化集中在 `unitree_setup.sh`，工作区编译和常用操作集中在 `go2.sh`。

第一次编译或改过代码后，在仓库目录运行：

```bash
./go2.sh build
```

它会用 `colcon` 做 Release 编译。后面我把网络选择、传感器检查、标定、SLAM 和实机系统启动都收进同一个脚本，也加了进程检查，避免重复开 SLAM。

常用入口是：

```bash
./go2.sh connect
./go2.sh check
./go2.sh calibrate-imu
./go2.sh slam
```

## 有线和无线怎么连

有线时，电脑接 Go2 后部网口。常见地址是电脑 `192.168.123.100`、Go2 `192.168.123.18`。`./go2.sh connect` 会先检查有线是否可达；有线通的时候就用有线。

最直接的无线方式是电脑加入 Go2 的 `Go2-Explore` 热点。Go2 是 `192.168.76.1`，电脑设为 `192.168.76.2/24`，不设默认网关。这个热点不提供互联网。运行：

```bash
GO2_FORCE_WIFI=1 ./go2.sh connect
GO2_FORCE_WIFI=1 ./go2.sh check
```

这里 `GO2_FORCE_WIFI=1` 的作用是让 CycloneDDS 明确走 Wi-Fi；即使网线还插着，也不会误走有线接口。网络能 ping 通只代表 IP 通，最后还是要看电脑能不能实际收到 ROS 话题样本。

## `~/go2-wireless` 是怎么搭的

最开始碰到的问题是：Go2 自己的 `eth0` 上能看到传感器 DDS 话题，直接从 `wlan0` 无线访问时却收不到。于是单独做了 `~/go2-wireless`，让狗端运行一个轻量 DDS 中继。

中继程序 `native_wifi_relay.py` 在 `eth0` 收到 IMU、点云、机身状态等数据，再从 `wlan0` 按原来的话题名和类型发出去。Sport API 请求走反方向：中继从 `wlan0` 收到电脑发来的 `/api/sport/request`，再从 `eth0` 发给狗上的 Sport API。两条方向分开处理，请求不会被转回 Wi-Fi 形成回环。

中继由 `go2-wifi-relay.service` 管理，开机后可以自动拉起，异常退出也会重启。`go2-fastdds-wifi.xml` 给狗端 DDS 配置 `eth0` 和 `wlan0` 两个接口。电脑端的 `ros-wifi-env.sh` 则把 ROS 2 Domain 设为 0、CycloneDDS 固定在无线网卡，并把 `192.168.76.1` 设成发现节点。

在狗上通过网线检查中继：

```bash
ssh unitree@192.168.123.18
systemctl --user status go2-wifi-relay.service
journalctl --user-unit=go2-wifi-relay -f
```

当时也做过一个 8 秒的狗端无线接口探测：没开中继时，`wlan0` 收不到选定的传感器话题；开中继后，五类话题都有样本。这个结果证明狗端无线接口在转发，电脑端仍要用 `./go2.sh check` 确认端到端收到了数据。

`~/go2-wireless` 还放了另一种连接方案：电脑开 `Go2-PC` 热点并保留校园网，Go2 的 `wlan0` 加入电脑热点，电脑和狗使用 `192.168.77.1/24`、`192.168.77.18/24`。对应脚本是 `pc-hotspot-up.sh`、`connect-go2-to-pc.sh` 和 `pc-hotspot-radio-fix.sh`。这套方案让电脑不用断开校园 Wi-Fi；它和“电脑加入 Go2-Explore”是两种连接拓扑，配置时按实际使用的那一种来。

## IMU 校准和 SLAM

L1 的 IMU 校准是通过 ROS 2 节点完成的。开始前脚本会确认 `/utlidar/imu` 数据发布正常，也会检查狗端能看到 `/api/sport/request` 订阅者。标定大约 37 秒：开始时让狗调整姿态，静止采集偏置，然后原地旋转约 20 秒，最后把结果写到桌面的 `~/Desktop/imu_calib_data.yaml`。如果已有旧文件，脚本会先备份。

实际操作时运行：

```bash
./go2.sh calibrate-imu
```

校准后，传感器转换节点会加载这个文件。IMU 校准跑通后，我用 Point-LIO 启动独立 SLAM：

```bash
./go2.sh slam
```

它先确认 IMU 和点云都收到实际样本，再启动传感器转换、Point-LIO 和 RViz。运行时可以查看点云、里程计和轨迹。`Ctrl+C` 退出。这个入口只跑 SLAM，不会同时启动局部规划或让狗自主走动。

## 外参和回放验证

为了让雷达和机身坐标系对得上，我还加了几项离线工具：`capture_extrinsics.py` 录 IMU、点云和机身状态，`estimate_extrinsics.py` 估计旋转参数，`validate_extrinsics_replay.py` 用录制数据回放比较 Point-LIO 结果。回放会使用独立 ROS Domain，不会给实机发送运动命令。

当前配置采用标定得到的雷达与陀螺仪旋转参数；加速度计坐标约定还没有验证。Point-LIO 的 `use_imu_as_input: false` 不会关闭 `mapping.imu_en`，因此回放仍会使用转换节点输出的加速度；不要把它称为 gyro-only。录制数据对照中，`use_imu_as_input: false` 的运动段姿态 RMS 约为 `13.4°`、最终误差约 `8.9°`；设为 `true` 后分别约为 `22.1°` 和 `48.9°`。这组结果只支持保留 `false`，并没有验证加速度外参。

时间差也试过约 `6 ms` 的候选值，但独立回放没有显示它更好，所以当前仍设为 `0.0`。平移外参没有独立测准，沿用仓库原有值。后续有更可靠的采集和验证结果，再更新这些配置。

## 现在跑到哪一步

目前已经完成仓库编译、实机通信检查、L1 IMU 校准和 Point-LIO SLAM 实机运行。无线中继和两种连接拓扑也整理好了，传感器数据的无线转发有端点与样本检查。

接下来主要是重复跑同一路线看地图稳定性，继续验证传感器参数，再启动局部规划和低速目标点跟随。真实运动控制要在场地安全、有人能接管的条件下单独验证；发现 `/api/sport/request` 的订阅者，只能说明请求链路可见，不能当成狗已经按指令运动的证据。

## 相关文件

- `go2.sh`：网络选择、通信检查、编译、IMU 校准、SLAM 和实机系统入口。
- `unitree_setup.sh`：ROS 2 Foxy、CycloneDDS 网卡和无线 Peer 配置。
- `docs/Go2实机操作手册.md`：按步骤操作的详细版本。
- `docs/Go2无线DDS转发需求.md`：无线 DDS 的话题方向与验收方式。
- `src/utilities/transform_sensors/config/l1_extrinsics.yaml`：当前 L1 旋转标定参数。
- `src/slam/point_lio_unilidar/launch/mapping_utlidar.launch`：实机 Point-LIO 启动配置。
- `~/go2-wireless/README.md`、`native_wifi_relay.py`：狗端中继的说明和实现。
