"""Exercise the real-robot waypoint request path on an isolated ROS domain."""

import json
import math
import os
from pathlib import Path
import subprocess
import time
import unittest

try:
    import rclpy
    from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
    from nav_msgs.msg import Odometry, Path as NavPath
    from sensor_msgs.msg import Joy, PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
    from tf2_ros import StaticTransformBroadcaster
    from unitree_api.msg import Request
    from unitree_go.msg import LowState
    import struct
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, "ROS 2 workspace is not sourced")
class WaypointControlTest(unittest.TestCase):
    def setUp(self):
        self.previous_env = {key: os.environ.get(key) for key in (
            "ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION", "CYCLONEDDS_URI")}
        os.environ.update(
            ROS_DOMAIN_ID="91", ROS_LOCALHOST_ONLY="0",
            RMW_IMPLEMENTATION="rmw_cyclonedds_cpp",
            CYCLONEDDS_URI=(
                '<CycloneDDS><Domain><General><Interfaces>'
                '<NetworkInterface name="lo" multicast="true" />'
                '</Interfaces></General></Domain></CycloneDDS>'))
        rclpy.init()
        self.node = rclpy.create_node("waypoint_control_test")
        self.odom_pub = self.node.create_publisher(Odometry, "/state_estimation", 10)
        self.path_pub = self.node.create_publisher(NavPath, "/path", 10)
        self.goal_pub = self.node.create_publisher(PointStamped, "/goal_point", 10)
        self.route_waypoint_pub = self.node.create_publisher(PointStamped, "/way_point", 10)
        self.joy_pub = self.node.create_publisher(Joy, "/joy", 10)
        self.remote_pub = self.node.create_publisher(LowState, "/lowstate", 10)
        self.collision_pub = self.node.create_publisher(
            PointCloud2,
            "/local_planner/collision_obstacles", 2)
        self.collision_points = []
        self.publish_collision = True
        self.remote_axis = 0.0
        self.publish_remote = True
        self.requests = []
        self.request_sub = self.node.create_subscription(
            Request, "/api/sport/request", self.requests.append, 10)
        self.tf = StaticTransformBroadcaster(self.node)
        executable = (Path(__file__).resolve().parents[1] / "install/local_planner/lib/"
                      "local_planner/pathFollower")
        self.process = subprocess.Popen(
            [str(executable), "--ros-args", "-r", "/way_point:=/goal_point",
             "-p", "is_real_robot:=true",
             "-p", "goalCloseDis:=0.2", "-p", "maxWaypointSpeed:=0.35",
             "-p", "waypointProgressTimeout:=2.0"],
            env=os.environ.copy(), stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        self.position = 0.0
        self.publish_path = True
        self.path_points = [(0.0, 0.0), (1.0, 0.0)]
        transform = TransformStamped()
        transform.header.frame_id = "map"
        transform.child_frame_id = "camera_init"
        transform.transform.rotation.z = math.sin(math.pi / 4)
        transform.transform.rotation.w = math.cos(math.pi / 4)
        self.tf.sendTransform(transform)
        self.pump(0.8)

    def tearDown(self):
        self.process.terminate()
        try:
            self.process.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.communicate()
        self.node.destroy_node()
        rclpy.shutdown()
        for key, value in self.previous_env.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value

    def pump(self, seconds, repeat_route_waypoint=None):
        deadline = time.monotonic() + seconds
        next_goal_time = time.monotonic()
        while time.monotonic() < deadline:
            stamp = self.node.get_clock().now().to_msg()
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = "camera_init"
            odom.pose.pose.position.x = self.position
            odom.pose.pose.orientation.w = 1.0
            self.odom_pub.publish(odom)
            if self.publish_collision:
                header = Header(stamp=stamp, frame_id="camera_init")
                self.collision_pub.publish(point_cloud2.create_cloud_xyz32(header, self.collision_points))
            path = NavPath()
            path.header.stamp = stamp
            path.header.frame_id = "vehicle"
            for x, y in self.path_points:
                entry = PoseStamped()
                entry.pose.position.x = x
                entry.pose.position.y = y
                path.poses.append(entry)
            if self.publish_path:
                self.path_pub.publish(path)
            if self.publish_remote:
                remote = LowState()
                remote.wireless_remote = list(
                    b"\x55\xaa\x00\x00" + struct.pack("<fff", 0, 0, 0)
                    + bytes(4) + struct.pack("<f", self.remote_axis) + bytes(16))
                self.remote_pub.publish(remote)
            if repeat_route_waypoint is not None and time.monotonic() >= next_goal_time:
                repeat_route_waypoint.header.stamp = stamp
                self.route_waypoint_pub.publish(repeat_route_waypoint)
                next_goal_time += 0.2
            rclpy.spin_once(self.node, timeout_sec=0.04)

    def test_goal_transform_motion_and_single_stop(self):
        joy = Joy()
        joy.axes = [0.0, 0.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0]
        self.joy_pub.publish(joy)
        self.pump(0.5)
        self.assertEqual([], self.requests, "autonomy must wait for a goal")

        goal = PointStamped()
        goal.header.frame_id = "map"
        goal.point.y = 1.0  # +90 degree map transform puts this 1 m ahead in camera_init.
        self.goal_pub.publish(goal)
        self.pump(1.0, repeat_route_waypoint=goal)
        moves = [req for req in self.requests if req.header.identity.api_id == 1008]
        self.assertTrue(moves, "waypoint ahead should command forward motion")
        self.assertLessEqual(len(moves), 12, "waypoint Move requests should stay near 10 Hz")
        self.assertTrue(any(0 < json.loads(req.parameter)["x"] <= 0.35 for req in moves))
        self.assertEqual([], [req for req in self.requests
                              if req.header.identity.api_id == 1003],
                         "5 Hz route waypoint updates must not interrupt local path tracking")

        self.position = 1.0
        self.pump(1.0)
        stops = [req for req in self.requests if req.header.identity.api_id == 1003]
        self.assertEqual(1, len(stops), "goal completion should release control once")
        count = len(self.requests)
        self.pump(0.5)
        self.assertEqual(count, len(self.requests), "no idle command stream after arrival")

    def test_replacement_goal_waits_for_path_and_resumes(self):
        joy = Joy()
        joy.axes = [0.0, 0.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0]
        self.joy_pub.publish(joy)
        goal = PointStamped()
        goal.header.frame_id = "map"
        goal.point.y = 1.0
        self.goal_pub.publish(goal)
        self.pump(0.6)
        self.assertTrue(any(r.header.identity.api_id == 1008 for r in self.requests))

        # Exercise multiple replacements without requiring another click to resume.
        for distance in (1.5, 2.0):
            # The planner can either be silent or publish a one-pose blocked path.
            self.publish_path = distance == 2.0
            self.path_points = [(0.0, 0.0)]
            goal.point.y = distance
            goal.header.stamp = self.node.get_clock().now().to_msg()
            self.goal_pub.publish(goal)
            before = len(self.requests)
            self.pump(0.3)
            self.assertEqual([1003], [r.header.identity.api_id for r in self.requests[before:]],
                             "replacement must stop once while waiting for its path")
            stopped = len(self.requests)
            self.pump(0.2)
            self.assertEqual(stopped, len(self.requests), "must not repeat StopMove while waiting")
            self.publish_path = True
            self.path_points = [(0.0, 0.0), (1.0, 0.0)]
            self.pump(0.4)
            self.assertTrue(any(r.header.identity.api_id == 1008 for r in self.requests[stopped:]),
                            "replacement goal was cancelled before its path arrived")

    def test_replacement_without_path_times_out(self):
        joy = Joy()
        joy.axes = [0.0, 0.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0]
        self.joy_pub.publish(joy)
        goal = PointStamped()
        goal.header.frame_id = "map"
        goal.point.y = 1.0
        self.goal_pub.publish(goal)
        self.pump(0.6)
        self.assertTrue(any(r.header.identity.api_id == 1008 for r in self.requests))
        self.publish_path = False
        goal.point.y = 2.0
        self.goal_pub.publish(goal)
        self.pump(2.5)
        self.assertEqual(1, sum(r.header.identity.api_id == 1003 for r in self.requests))
        stopped = len(self.requests)
        self.publish_path = True
        self.pump(0.5)
        self.assertEqual(stopped, len(self.requests), "expired replacement must not restart")

    def test_turn_is_bounded_and_lost_path_stops_once(self):
        self.path_points = [(0.0, 0.0), (0.0, 1.0)]
        joy = Joy()
        joy.axes = [0.0, 0.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0]
        self.joy_pub.publish(joy)
        goal = PointStamped()
        goal.header.frame_id = "map"
        goal.point.y = 1.0
        self.goal_pub.publish(goal)
        self.pump(1.0)
        moves = [req for req in self.requests if req.header.identity.api_id == 1008]
        self.assertTrue(moves)
        self.assertTrue(all(abs(json.loads(req.parameter)["z"]) <= 0.5 for req in moves))
        self.assertTrue(all(abs(json.loads(req.parameter)["x"]) < 0.01 for req in moves))

        self.path_points = [(0.0, 0.0)]
        self.pump(0.6)
        stops = [req for req in self.requests if req.header.identity.api_id == 1003]
        self.assertEqual(1, len(stops))
        count = len(self.requests)
        self.pump(0.5)
        self.assertEqual(count, len(self.requests))

        self.path_points = [(0.0, 0.0), (0.0, 1.0)]
        self.pump(0.5)
        self.assertEqual(count, len(self.requests),
                         "restored path must not reactivate an old goal")

    def test_no_progress_aborts_rotation(self):
        self.path_points = [(0.0, 0.0), (0.0, 1.0)]
        joy = Joy()
        joy.axes = [0.0, 0.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0]
        self.joy_pub.publish(joy)
        goal = PointStamped()
        goal.header.frame_id = "map"
        goal.point.y = 1.0
        self.goal_pub.publish(goal)
        self.pump(3.0, repeat_route_waypoint=goal)
        self.assertTrue(any(req.header.identity.api_id == 1008 for req in self.requests))
        self.assertEqual(1, sum(req.header.identity.api_id == 1003
                                for req in self.requests))
        count = len(self.requests)
        self.pump(0.5)
        self.assertEqual(count, len(self.requests))

    def test_remote_takeover_releases_without_stop_request(self):
        joy = Joy()
        joy.axes = [0.0, 0.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0]
        self.joy_pub.publish(joy)
        goal = PointStamped()
        goal.header.frame_id = "map"
        goal.point.y = 1.0
        self.goal_pub.publish(goal)
        self.pump(0.8, repeat_route_waypoint=goal)
        self.assertTrue(any(req.header.identity.api_id == 1008 for req in self.requests))

        self.remote_axis = 0.7
        self.pump(0.2, repeat_route_waypoint=goal)
        count = len(self.requests)
        self.assertEqual(0, sum(req.header.identity.api_id == 1003
                                for req in self.requests))
        self.remote_axis = 0.0
        self.pump(0.6, repeat_route_waypoint=goal)
        self.assertEqual(count, len(self.requests))

    def test_collision_blocks_motion_and_releases_with_one_stop(self):
        self.path_points = [(0.0, 0.0), (0.0, 1.0)]
        joy = Joy()
        joy.axes = [0.0, 0.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0]
        self.joy_pub.publish(joy)
        goal = PointStamped()
        goal.header.frame_id = "map"
        goal.point.y = 1.0
        self.goal_pub.publish(goal)
        self.pump(0.7)
        self.assertTrue(any(r.header.identity.api_id == 1008 for r in self.requests))
        self.collision_points = [(0.39, 0.0, 0.0)]
        self.pump(0.25)
        count = len(self.requests)
        self.pump(0.45)
        self.assertEqual(count, len(self.requests), "blocked turn must not repeatedly issue StopMove")
        self.assertEqual(1, sum(r.header.identity.api_id == 1003 for r in self.requests))

    def test_obstacle_reduces_speed_without_stop_start_chatter(self):
        self.collision_points = [(0.46, 0.0, 0.0)]
        joy = Joy()
        joy.axes = [0.0, 0.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0]
        self.joy_pub.publish(joy)
        goal = PointStamped()
        goal.header.frame_id = "map"
        goal.point.y = 1.0
        self.goal_pub.publish(goal)
        self.pump(1.1)
        moves = [json.loads(r.parameter) for r in self.requests if r.header.identity.api_id == 1008]
        self.assertGreater(len(moves), 4)
        self.assertTrue(all(0.01 < r['x'] < 0.30 for r in moves[-4:]))
        self.assertLess(max(r['x'] for r in moves[-4:])-min(r['x'] for r in moves[-4:]), 0.03)
        self.assertFalse(any(r.header.identity.api_id == 1003 for r in self.requests))

    def test_stale_collision_cloud_stops_motion(self):
        joy = Joy()
        joy.axes = [0.0, 0.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0]
        self.joy_pub.publish(joy)
        goal = PointStamped()
        goal.header.frame_id = "map"
        goal.point.y = 1.0
        self.goal_pub.publish(goal)
        self.pump(0.7)
        self.assertTrue(any(r.header.identity.api_id == 1008 for r in self.requests))
        self.publish_collision = False
        self.pump(0.8)
        count = len(self.requests)
        moves_before_resume = sum(
            request.header.identity.api_id == 1008 for request in self.requests)
        self.pump(0.3)
        self.assertEqual(count, len(self.requests))
        self.assertEqual(1, sum(r.header.identity.api_id == 1003 for r in self.requests))
        self.publish_collision = True
        self.pump(0.8)
        self.assertGreater(
            sum(r.header.identity.api_id == 1008 for r in self.requests),
            moves_before_resume,
            "fresh obstacle data should release the stale-cloud stop")

    def test_autonomy_continues_without_fresh_remote_state(self):
        joy = Joy()
        joy.axes = [0.0, 0.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0]
        self.joy_pub.publish(joy)
        goal = PointStamped()
        goal.header.frame_id = "map"
        goal.point.y = 1.0
        self.goal_pub.publish(goal)
        self.pump(0.7)
        moves_before_timeout = sum(r.header.identity.api_id == 1008 for r in self.requests)
        self.assertGreater(moves_before_timeout, 0)
        self.publish_remote = False
        self.pump(0.8)
        moves_after_timeout = sum(r.header.identity.api_id == 1008 for r in self.requests)
        self.assertGreater(moves_after_timeout, moves_before_timeout)
        self.assertFalse(any(r.header.identity.api_id == 1003 for r in self.requests))

    def test_missing_remote_state_blocks_motion(self):
        self.publish_remote = False
        self.pump(0.7)
        joy = Joy()
        joy.axes = [0.0, 0.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0]
        joy.axes[2] = 0.0  # Manual mode still requires a fresh remote state.
        self.joy_pub.publish(joy)
        goal = PointStamped()
        goal.header.frame_id = "map"
        goal.point.y = 1.0
        self.goal_pub.publish(goal)
        self.pump(1.0)
        self.assertEqual([], self.requests)


if __name__ == "__main__":
    unittest.main()
