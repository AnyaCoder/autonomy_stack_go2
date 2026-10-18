# Go2 Wi-Fi Relay

The native relay uses two processes and three isolated DDS domains. `--source`
reads the Go2 wired DDS domain and writes typed samples to a loopback-only local
domain. `--wifi` reads those local topics and publishes them on the Go2 Wi-Fi
DDS domain. Sport requests travel back through a separate local topic. This
keeps source readers from seeing the relay's outgoing writers and avoids a DDS
feedback loop. `go2_wifi_graph_announcer` registers ROS graph endpoints, while
`go2-wifi-stack.target` manages all three processes together.

## Topic map

| Direction | DDS topic | ROS topic | Type |
| --- | --- | --- | --- |
| Go2 to PC | `rt/utlidar/imu` | `/utlidar/imu` | `sensor_msgs/msg/Imu` |
| Go2 to PC | `rt/utlidar/cloud` | `/utlidar/cloud` | `sensor_msgs/msg/PointCloud2` |
| Go2 to PC | `rt/sportmodestate` | `/sportmodestate` | `unitree_go/msg/SportModeState` |
| Go2 to PC | `rt/lowstate` | `/lowstate` | `unitree_go/msg/LowState` |
| PC to Go2 | `rt/go2/wifi/sport/request` | `/go2/wifi/sport/request` | `unitree_api/msg/Request` |
| PC to Go2 | `rt/api/sport/request` | `/api/sport/request` | `unitree_api/msg/Request` |

Native sensor/state forwarding is one way from `eth0` sources to Wi-Fi clients.
Sport requests use a dedicated Wi-Fi input and are written to the native Sport
API topic. The relay never subscribes to `rt/api/sport/request`, preventing a
robot-originated request from looping back into the robot.

## Build and service control

Run `build_on_go2.sh` on the robot after copying this directory to the same
relative location. It writes a staging binary ending in `.new`; replace the live
binary only after the build succeeds. Manage ingress, egress, and graph discovery together:

```sh
systemctl --user start go2-wifi-stack.target
systemctl --user stop go2-wifi-stack.target
systemctl --user status go2-wifi-stack.target
systemctl --user enable go2-wifi-stack.target
systemctl --user enable go2-wifi-relay-wifi.service
```

For startup without an interactive login, enable linger for the `unitree`
account (`sudo loginctl enable-linger unitree`). The stack target is wanted by
the user `default.target`; its Wants dependencies start ingress, Wi-Fi egress,
and graph discovery together.

The DDS descriptor source is in `relay_types.idl` and `unitree_relay.idl`;
generated C descriptors are kept under `generated/`.
