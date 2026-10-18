"""Offline localPlanner scenarios using the actual executable and loopback DDS."""
import math
import os
from pathlib import Path
import subprocess
import time
import unittest

import rclpy
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray


class LocalFootprintPlannerTest(unittest.TestCase):
    def setUp(self):
        keys = ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION', 'CYCLONEDDS_URI')
        self.previous = {key: os.environ.get(key) for key in keys}
        os.environ.update(ROS_DOMAIN_ID='91', ROS_LOCALHOST_ONLY='0',
            RMW_IMPLEMENTATION='rmw_cyclonedds_cpp', CYCLONEDDS_URI=(
                '<CycloneDDS><Domain><General><Interfaces>'
                '<NetworkInterface name="lo" multicast="true" />'
                '</Interfaces></General></Domain></CycloneDDS>'))
        rclpy.init()
        self.node = rclpy.create_node('footprint_planner_test')
        self.odom = self.node.create_publisher(Odometry, '/state_estimation', 5)
        self.terrain = self.node.create_publisher(PointCloud2, '/terrain_map', 5)
        self.paths = []
        self.obstacles = []
        self.footprints = []
        self.node.create_subscription(NavPath, '/path', self.paths.append, 5)
        self.node.create_subscription(PointCloud2, '/local_planner/collision_obstacles',
                                      self.obstacles.append, 5)
        self.node.create_subscription(MarkerArray, '/local_planner/collision_footprint',
                                      self.footprints.append, 1)
        executable = Path(__file__).resolve().parents[1] / 'install/local_planner/lib/local_planner/localPlanner'
        paths = Path(__file__).resolve().parents[1] / 'src/base_autonomy/local_planner/paths'
        self.process = subprocess.Popen([str(executable), '--ros-args',
            '-p', f'pathFolder:={paths}', '-p', 'useTerrainAnalysis:=true',
            '-p', 'autonomyMode:=true', '-p', 'autonomySpeed:=0.35',
            '-p', 'sensorOffsetX:=0.3', '-p', 'twoWayDrive:=false',
            '-p', 'goalX:=2.0', '-p', 'goalY:=0.0'],
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)

    def tearDown(self):
        self.process.terminate()
        try:
            self.process.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.communicate()
        self.node.destroy_node()
        rclpy.shutdown()
        for key, value in self.previous.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value

    def scene(self, width, yaw=0.0):
        self.paths.clear()
        self.obstacles.clear()
        self.footprints.clear()
        c, s = math.cos(yaw), math.sin(yaw)
        deadline = time.monotonic() + 1.2
        while time.monotonic() < deadline:
            stamp = self.node.get_clock().now().to_msg()
            odom = Odometry()
            odom.header = Header(stamp=stamp, frame_id='camera_init')
            odom.pose.pose.position.x = 0.3*c
            odom.pose.pose.position.y = 0.3*s
            odom.pose.pose.orientation.z = math.sin(yaw/2)
            odom.pose.pose.orientation.w = math.cos(yaw/2)
            self.odom.publish(odom)
            points = [(c*x-s*y, s*x+c*y, 0.0, 0.5)
                      for x in [i*0.02 for i in range(-50,101)]
                      for y in (-width/2,width/2)]
            fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                      for i,name in enumerate(('x','y','z','intensity'))]
            self.terrain.publish(point_cloud2.create_cloud(
                Header(stamp=stamp, frame_id='camera_init'),fields,points))
            rclpy.spin_once(self.node, timeout_sec=0.03)
        self.assertIsNone(self.process.poll(), 'localPlanner exited')
        self.assertTrue(self.paths, 'planner did not publish paths')
        self.assertTrue(self.obstacles, 'collision obstacle cloud missing')
        self.assertEqual('camera_init',self.obstacles[-1].header.frame_id)
        self.assertTrue(self.footprints, 'RViz collision footprint missing')
        by_name = {m.ns: m for m in self.footprints[-1].markers}
        for name, length, width in [('body_envelope',0.70,0.40),
                                    ('measurement_margin',0.74,0.44),
                                    ('collision_envelope',0.80,0.50)]:
            marker = by_name[name]
            self.assertEqual('camera_init', marker.header.frame_id)
            self.assertAlmostEqual(0.0, marker.pose.position.x, places=5)
            self.assertAlmostEqual(0.0, marker.pose.position.y, places=5)
            self.assertAlmostEqual(math.sin(yaw/2),marker.pose.orientation.z,places=5)
            self.assertAlmostEqual(math.cos(yaw/2),marker.pose.orientation.w,places=5)
            self.assertAlmostEqual(length,max(p.x for p in marker.points)-min(p.x for p in marker.points))
            self.assertAlmostEqual(width,max(p.y for p in marker.points)-min(p.y for p in marker.points))
        return self.paths[-1]

    def test_sixty_cm_corridor_passes(self):
        path = self.scene(0.60)
        self.assertGreater(len(path.poses),1)
        self.assertGreater(path.poses[-1].pose.position.x,0.3)
        self.assertLess(abs(path.poses[-1].pose.position.y),0.05)

    def test_forty_eight_cm_corridor_blocks(self):
        path = self.scene(0.48)
        self.assertEqual(1,len(path.poses))
        self.assertEqual(0.0,path.poses[0].pose.position.x)

    def test_rotated_world_and_front_lidar_offset(self):
        # The forward world-X goal is now to the right of the robot: use an
        # independent no-obstacle scene to verify world-frame cloud coordinates.
        path = self.scene(0.60, math.pi/2)
        points = list(point_cloud2.read_points(self.obstacles[-1],field_names=('x','y')))
        self.assertTrue(any(abs(abs(x)-0.30)<0.001 for x,y in points))
        self.assertTrue(all(abs(x)<0.301 for x,y in points))
        self.assertEqual('vehicle',path.header.frame_id)


if __name__ == '__main__':
    unittest.main()
