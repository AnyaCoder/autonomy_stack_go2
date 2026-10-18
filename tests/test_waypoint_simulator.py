"""Run waypoint following against the repository's vehicle dynamics simulator."""

import os
from pathlib import Path
import subprocess
import time
import unittest
import xml.etree.ElementTree as ET

try:
    import rclpy
    from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
    from nav_msgs.msg import Odometry, Path as NavPath
    from sensor_msgs.msg import Joy
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, "ROS 2 workspace is not sourced")
class WaypointSimulatorTest(unittest.TestCase):
    def test_vehicle_reaches_goal_and_stays_stopped(self):
        root = Path(__file__).resolve().parents[1]
        launch = ET.parse(root / 'src/base_autonomy/vehicle_simulator/launch/system_simulation_with_route_planner.launch')
        follower_goal = launch.find(".//include/arg[@name='pathFollowerGoalTopic']")
        self.assertIsNotNone(follower_goal, "FAR simulation must remap the follower's final goal")
        goal_topic = follower_goal.attrib['value']
        self.assertEqual('/goal_point', goal_topic)
        env = os.environ.copy()
        env.update(
            ROS_DOMAIN_ID="91", ROS_LOCALHOST_ONLY="0",
            RMW_IMPLEMENTATION="rmw_cyclonedds_cpp",
            CYCLONEDDS_URI=(
                '<CycloneDDS><Domain><General><Interfaces>'
                '<NetworkInterface name="lo" multicast="true" />'
                '</Interfaces></General></Domain></CycloneDDS>'))
        processes = [
            subprocess.Popen(
                [str(root / "install/vehicle_simulator/lib/vehicle_simulator/vehicleSimulator")],
                env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
            subprocess.Popen(
                [str(root / "install/local_planner/lib/local_planner/pathFollower"),
                 "--ros-args", "-r", f"/way_point:={goal_topic}", "-p", "is_real_robot:=false", "-p", "goalCloseDis:=0.2"],
                env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
        ]
        previous = {key: os.environ.get(key) for key in env if key in (
            "ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION", "CYCLONEDDS_URI")}
        os.environ.update({key: env[key] for key in previous})
        rclpy.init()
        node = rclpy.create_node("waypoint_simulator_test")
        position = [0.0]
        odom_frame = ["map"]
        commands = []
        def on_odom(msg):
            position[0] = msg.pose.pose.position.x
            odom_frame[0] = msg.header.frame_id
        node.create_subscription(Odometry, "/state_estimation", on_odom, 10)
        node.create_subscription(TwistStamped, "/cmd_vel", commands.append, 10)
        from sensor_msgs.msg import PointCloud2
        collision_pub = node.create_publisher(PointCloud2, "/local_planner/collision_obstacles", 2)
        path_pub = node.create_publisher(NavPath, "/path", 10)
        goal_pub = node.create_publisher(PointStamped, goal_topic, 10)
        route_pub = node.create_publisher(PointStamped, "/way_point", 10)
        joy_pub = node.create_publisher(Joy, "/joy", 10)
        try:
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and node.count_subscribers("/path") == 0:
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertGreater(node.count_subscribers("/path"), 0)

            joy = Joy()
            joy.axes = [0.0, 0.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0]
            joy_pub.publish(joy)
            goal = PointStamped()
            goal.header.frame_id = "map"
            goal.point.x = 1.0
            goal_pub.publish(goal)

            next_route_time = time.monotonic()
            deadline = time.monotonic() + 8.0
            while time.monotonic() < deadline and position[0] < 0.8:
                path = NavPath()
                path.header.frame_id = "vehicle"
                path.poses = [PoseStamped(), PoseStamped()]
                path.poses[1].pose.position.x = max(0.05, 1.0 - position[0])
                path_pub.publish(path)
                if time.monotonic() >= next_route_time:
                    route = PointStamped()
                    route.header = Header(frame_id="map", stamp=node.get_clock().now().to_msg())
                    route.point.x = min(1.0, position[0]+0.5)
                    route_pub.publish(route)
                    next_route_time += 0.2
                collision_pub.publish(point_cloud2.create_cloud_xyz32(
                    Header(frame_id=odom_frame[0]), []))
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertGreaterEqual(position[0], 0.8, "simulated vehicle did not reach goal")
            reached_at = position[0]
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertLess(abs(position[0] - reached_at), 0.05,
                            "simulated vehicle continued moving after arrival")
            self.assertTrue(commands)
            self.assertEqual(0.0, commands[-1].twist.linear.x)
        finally:
            node.destroy_node()
            rclpy.shutdown()
            for process in processes:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            for key, value in previous.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value


if __name__ == "__main__":
    unittest.main()
