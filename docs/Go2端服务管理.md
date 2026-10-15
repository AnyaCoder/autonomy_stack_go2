# Go2 端服务管理

Go2 的自定义运行文件集中在 `~/go2`。systemd 用户服务定义保留在 `~/.config/systemd/user`，这是 systemd 的标准位置；服务日志由 journal 管理。

## 服务分组

| 分组 | 服务 | 用途 |
|---|---|---|
| `wireless` | `go2-wifi-relay` | Go2 传感器无线 DDS 中继；运动请求转发因回环故障暂停 |
| `sensors` | `go2-sensors` | L1 点云/里程计转 ROS 2 |
| `sensors` | `go2-front-camera` | 机头相机只读图像服务 |
| `sensors` | `go2-lowstate` | 关节与 IMU 只读状态服务 |
| `sensors` | `go2-realsense` | D435i 图像话题 |
| `sensors` | `go2-realsense-preview` | D435i 彩色与深度低频预览 |
| `sensors` | `go2-laserscan` | 点云转 `/scan` |

这些服务通过 `~/go2/go2-services.sh` 管理：

```bash
~/go2/go2-services.sh status
~/go2/go2-services.sh restart sensors
~/go2/go2-services.sh logs wireless
```

支持 `status`、`start`、`stop`、`restart`、`enable`、`disable` 和 `logs`，分组为 `all`、`wireless`、`sensors`。`go2-slam-inputs` 已停用并清理，不属于当前服务集。

服务定义引用的实现和配置位于 `~/go2/services/voxel_ws`；DDS runtime、Unitree Python SDK 和无线 relay 分别位于 `~/go2/runtime`、`~/go2/vendor`、`~/go2/relay`。地图保存在 `~/go2/data`，迁移备份和诊断数据保存在 `~/go2/archive`。
