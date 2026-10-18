"""Align Point-LIO's initial map frame with a stationary ground plane."""

# Copyright 2026 The Go2 Autonomy Stack contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformBroadcaster
from unitree_go.msg import SportModeState


def ground_slope(points):
    points = np.asarray(points, dtype=float)
    points = points[np.isfinite(points).all(axis=1)]
    points = points[(points[:, 0] > 0.3) & (points[:, 0] < 3.0) &
                    (np.abs(points[:, 1]) < 1.5) &
                    (points[:, 2] > -0.8) & (points[:, 2] < 0.15)]
    if len(points) < 100:
        return None

    xy1 = np.column_stack((points[:, :2], np.ones(len(points))))
    z = points[:, 2]
    rng = np.random.default_rng(7)
    best = np.zeros(len(points), dtype=bool)
    for _ in range(80):
        ids = rng.choice(len(points), 3, replace=False)
        try:
            slope = np.linalg.solve(xy1[ids], z[ids])
        except np.linalg.LinAlgError:
            continue
        if np.linalg.norm(slope[:2]) > 0.25:
            continue
        inliers = np.abs(z - xy1 @ slope) < 0.035
        if inliers.sum() > best.sum():
            best = inliers
    for _ in range(3):
        if best.sum() < 3:
            return None
        slope = np.linalg.lstsq(xy1[best], z[best], rcond=None)[0]
        best = np.abs(z - xy1 @ slope) < 0.035
    if best.sum() < max(60, int(0.35 * len(points))):
        return None
    residual = np.abs(z[best] - xy1[best] @ slope)
    if np.quantile(residual, 0.95) > 0.03:
        return None
    return slope[:2]


def leveling_quaternion(slope):
    normal = np.array([-slope[0], -slope[1], 1.0], dtype=float)
    normal /= np.linalg.norm(normal)
    axis = np.cross(normal, [0.0, 0.0, 1.0])
    cosine = 1.0 + normal[2]
    quaternion = np.r_[axis, cosine]
    quaternion /= np.linalg.norm(quaternion)
    return quaternion


class LevelMap(Node):
    def __init__(self):
        super().__init__('level_map')
        self.broadcaster = TransformBroadcaster(self)
        self.slopes = []
        self.started = time.monotonic()
        self.published = False
        self.stationary = False
        self.rotation = leveling_quaternion(np.zeros(2))
        self.subscription = self.create_subscription(
            PointCloud2, '/registered_scan', self.cloud_callback,
            qos_profile_sensor_data)
        self.state_subscription = self.create_subscription(
            SportModeState, '/sportmodestate', self.state_callback,
            qos_profile_sensor_data)
        self.timer = self.create_timer(0.1, self.publish_current_transform)

    def state_callback(self, message):
        self.stationary = (np.linalg.norm(message.velocity) < 0.05 and
                           abs(message.yaw_speed) < 0.05 and
                           np.linalg.norm(message.imu_state.gyroscope) < 0.05)
        if not self.stationary:
            self.slopes.clear()

    def publish_transform(self, slope):
        if self.published:
            return
        self.rotation = leveling_quaternion(slope)
        self.published = True
        self.publish_current_transform()
        tilt = math.degrees(math.atan(np.linalg.norm(slope)))
        self.get_logger().info(
            'Map ground leveling applied: %.2f deg, slopes x=%.4f y=%.4f'
            % (tilt, slope[0], slope[1]))

    def publish_current_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'camera_init'
        transform.transform.rotation.x = float(self.rotation[0])
        transform.transform.rotation.y = float(self.rotation[1])
        transform.transform.rotation.z = float(self.rotation[2])
        transform.transform.rotation.w = float(self.rotation[3])
        self.broadcaster.sendTransform(transform)
        if not self.published and time.monotonic() - self.started > 20:
            self.get_logger().warn(
                'Ground fit unavailable; using unlevelled map frame')
            self.publish_transform(np.zeros(2))

    def cloud_callback(self, message):
        if (self.published or not self.stationary or
                message.header.frame_id != 'camera_init'):
            return
        points = np.asarray(list(point_cloud2.read_points(
            message, field_names=('x', 'y', 'z'), skip_nans=True)))
        slope = ground_slope(points)
        if slope is None:
            return
        self.slopes.append(slope)
        if len(self.slopes) < 12:
            return
        estimates = np.asarray(self.slopes[-12:])
        median = np.median(estimates, axis=0)
        if np.max(np.linalg.norm(estimates - median, axis=1)) > 0.018:
            return
        self.publish_transform(median)


def main(args=None):
    rclpy.init(args=args)
    node = LevelMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
