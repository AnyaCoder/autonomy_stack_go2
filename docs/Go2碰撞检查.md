# Go2 碰撞检查

`./go2.sh run` 和 `./go2.sh run far` 共用 localPlanner/pathFollower 的矩形扫掠检查，平移和旋转检查默认开启。SLAM 不发送导航控制。

## 实体尺寸与余量

统一入口：`src/base_autonomy/local_planner/launch/local_planner.launch`。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| vehicleLength | 0.70 m | 机身及步态的前后包络初值 |
| vehicleWidth | 0.40 m | 机身及步态的左右包络初值 |
| collisionMeasurementMargin | 0.02 m | 按障碍测量误差 2 cm 的假设，每侧增加 |
| collisionTrackingMargin | 0.03 m | 每侧额外行走跟踪余量 |
| checkRotObstacle | true | 转身时检查前后角部扫过的区域 |
| collisionReactionTime | 0.20 s | 指令、感知、执行延迟的计算初值 |
| collisionDeceleration | 0.80 m/s² | 停止距离计算采用的减速度初值 |
| collisionAngularDeceleration | 1.0 rad/s² | 转向停止角计算采用的角减速度初值 |
| collisionCloudTimeout | 0.50 s | 障碍数据超过此接收间隔即停止导航输出 |

默认膨胀矩形为 **0.80 × 0.50 m**；连续扫掠采样另有至多约 5 mm 的数值覆盖余量。宽 60 cm 的直走廊可通过几何检查，但其中原地旋转可能受阻，取决于机身角部扫过的范围。

70 × 40 cm 是可调的 Go2 行走包络初值，并非对当前机器的实测标定。背部载荷、腿部横摆、实际停距、雷达遮挡和定位误差需要实机核实。2 cm 是输入误差假设，不等于把整套 SLAM/地形图的实际误差认证为 2 cm。

局部障碍高度阈值从 0.30 m 改为 **0.10 m**（相对地面高度）；旧阈值会放过 10–30 cm 的矮箱子或横杆。软地形代价起点为 0.05 m。低于 10 cm 的地形、雷达没有扫到的物体不在这个硬障碍集合中。

## RViz 显示

`run` 和 `run far` 均默认启用 `Local Collision Obstacles`：紫红色点来自 `/local_planner/collision_obstacles`，就是两种模式共同用于局部碰撞检查的障碍集合。显示只保留最新一帧，4 像素点大小只是方便观察，不表示障碍额外膨胀。这里没有统一封闭边界线；FAR 的红色轮廓是路线规划的另一种表示。

地形分析估计每处地面的高度，将每个点距地面的高度写入 intensity；局部规划器当前把该高度大于 0.10 m 的点作为障碍。地形模块还可能为缺少地面观测的区域生成虚拟障碍，外部添加的障碍和导航边界也会加入集合，因此紫红色点不全是雷达原始回波。

碰撞检查相当于把带余量的机身矩形沿运动轨迹扫过去：碰到紫红色点就限制该运动。障碍点本身不再统一向外画一条 2 cm 的膨胀线；测量和行走余量体现在机身碰撞框中。

`run` 和 `run far` 默认启用 `Go2 Collision Footprint`，订阅电脑本地的 `/local_planner/collision_footprint`：

- 蓝框：机身行走包络，默认 70 × 40 cm。
- 青框：加入 2 cm 测量余量，默认 74 × 44 cm。
- 橙框与透明填充：实际使用的碰撞包络，默认 80 × 50 cm。
- 白色箭头：机身正前方。

尺寸直接读取 localPlanner 的碰撞参数。位置使用扣除雷达安装偏移后的机身中心，方向使用碰撞检查相同的 yaw，位于当前雷达高度的水平截面；机身俯仰/横滚不会把平面碰撞框倾斜。这里显示的是二维碰撞足迹，不虚构尚未实现的三维碰撞高度。停止接收里程计后显示自动过期。减速预测的动态扫掠范围、约 5 mm 采样覆盖余量不属于这个静态框。

## 算法

1. 以 `vehicle` 为机身中心，使用已有 `sensorOffsetX/Y` 从前置雷达坐标换算位置，真实机器人启动默认 X 偏移 0.3 m。
2. localPlanner 对实际会发布的短轨迹逐段检查。机身矩形不随 `pathScale` 缩放；先检查从当前朝向转到候选方向的扫掠，再检查沿轨迹移动和改变朝向的扫掠。
3. 每个障碍点先经空间桶查询，再变换进矩形自身坐标检查。空间桶保留原点，不用体素中心替代障碍物位置。
4. 平移距离与角部旋转距离共同决定采样数，并用相邻采样间的位移界膨胀矩形，覆盖两个姿态之间的运动。
5. `/local_planner/collision_obstacles` 发布与规划器相同的障碍集合，包含地形分类后的障碍、附加障碍和导航边界。坐标系是当前 odometry frame；该话题由电脑本地产生，无线链路不需要增加狗端转发话题。
6. pathFollower 以当前实际位姿和将要下发的 `(vx, vy, wz)`，预测反应阶段与随后线性减速阶段的组合运动。既检查纯旋转，也检查边走边转。扫掠受阻时二分寻找可用速度比例，平移与旋转同比减速，避免反复全速加速再停止；当前包络已被占用、数据缺失或坐标系不匹配时，导航速度归零。
7. 延续现有控制权释放机制：仅在从运动转为停止时发送一次 StopMove；持续受阻不连续发送 StopMove。原厂遥控器接管后释放导航控制权；RViz 的显式 manual mode 延续原有人工直控语义，不经过导航避障。

硬碰撞判定不再使用旧的 0.55 m 圆形路径对应表。该表仅在显式启用 `useCost` 时用于软地形代价；默认不加载。RViz 的 `/free_paths` 只显示检查过的可执行短轨迹。

上游本地 terrainAnalysis 的扫描下采样分辨率改为 0.02 m，localPlanner 碰撞索引直接保留分类后的点，取消该索引原先的 0.2 m 二次重心下采样。地形高度分类、未知区域补障碍和 FAR 自己的图拓扑/栅格判定仍会影响可通行区域，不能由红线 padding 单独表示。

## 离线验证

```bash
g++ -std=c++14 -O2 -Wall -Wextra -pedantic \
  -Isrc/base_autonomy/local_planner/include \
  tests/test_footprint_collision.cpp -o /tmp/test_footprint_collision
/tmp/test_footprint_collision

source /opt/ros/foxy/setup.bash
source install/setup.bash
python3 tests/test_local_footprint_planner.py
python3 tests/test_waypoint_control.py
python3 tests/test_waypoint_simulator.py
```

ROS 测试沿用测试用的 loopback DDS 配置，不接入机器人。覆盖：直走窄走廊、转身角部碰撞、后角碰撞、连续运动中间的薄障碍、同时平移与旋转、负坐标、尺寸不随路径缩放，以及障碍阻挡后只停止一次、障碍流中断、遥控器接管。
