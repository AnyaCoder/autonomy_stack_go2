#!/usr/bin/env python3
"""Record message age for the real-robot SLAM pipeline.

This node is passive: it only subscribes to sensor and SLAM output topics and
writes one CSV row per received message. It never publishes motion or sensor
messages and does not store point-cloud payloads.
"""

import argparse
import csv
from datetime import datetime, timezone
from pathlib import Path
import signal
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2


TOPICS = (
    ("/utlidar/cloud", PointCloud2),
    ("/utlidar/imu", Imu),
    ("/utlidar/transformed_cloud", PointCloud2),
    ("/utlidar/transformed_raw_imu", Imu),
    ("/utlidar/transformed_imu", Imu),
    ("/state_estimation", Odometry),
    ("/registered_scan", PointCloud2),
)


def stamp_nanoseconds(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class SlamTimingRecorder(Node):
    def __init__(self, output_path):
        super().__init__("slam_timing_recorder")
        output_path = Path(output_path).expanduser()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = output_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._stream)
        self._writer.writerow(
            (
                "wall_time",
                "topic",
                "msg_stamp",
                "age_ms",
                "clock_offset_ms",
                "relative_age_ms",
                "inter_arrival_ms",
                "frame_id",
                "point_count",
                "position_x_m",
                "position_y_m",
                "position_z_m",
                "orientation_x",
                "orientation_y",
                "orientation_z",
                "orientation_w",
                "linear_velocity_x_mps",
                "linear_velocity_y_mps",
                "linear_velocity_z_mps",
                "angular_velocity_x_radps",
                "angular_velocity_y_radps",
                "angular_velocity_z_radps",
                "gyro_x_radps",
                "gyro_y_radps",
                "gyro_z_radps",
                "acceleration_x_mps2",
                "acceleration_y_mps2",
                "acceleration_z_mps2",
            )
        )
        self._stream.flush()
        self._output_path = output_path
        self._subscriptions = []
        self._baseline_offset_ns = {}
        self._last_receive_ns = {}
        self._record_count = 0
        for topic, message_type in TOPICS:
            self._subscriptions.append(
                self.create_subscription(
                    message_type,
                    topic,
                    lambda msg, topic=topic: self._record(topic, msg),
                    qos_profile_sensor_data,
                )
            )
        self.get_logger().info("Writing timing CSV: %s" % output_path)

    def _record(self, topic, message):
        receive_ns = time.time_ns()
        message_ns = stamp_nanoseconds(message.header.stamp)
        offset_ns = receive_ns - message_ns
        baseline_ns = self._baseline_offset_ns.setdefault(topic, offset_ns)
        previous_receive_ns = self._last_receive_ns.get(topic)
        self._last_receive_ns[topic] = receive_ns
        age_ms = offset_ns / 1_000_000.0
        relative_age_ms = (offset_ns - baseline_ns) / 1_000_000.0
        inter_arrival_ms = ""
        if previous_receive_ns is not None:
            inter_arrival_ms = "%.3f" % (
                (receive_ns - previous_receive_ns) / 1_000_000.0
            )
        point_count = ""
        if isinstance(message, PointCloud2):
            point_count = int(message.width) * int(message.height)
        measurements = [""] * 19
        if isinstance(message, Odometry):
            pose = message.pose.pose
            twist = message.twist.twist
            measurements[:13] = [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
                twist.angular.x,
                twist.angular.y,
                twist.angular.z,
            ]
        elif isinstance(message, Imu):
            measurements[13:] = [
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ]
        self._writer.writerow(
            (
                datetime.fromtimestamp(receive_ns / 1_000_000_000, timezone.utc).isoformat(),
                topic,
                "%.9f" % (message_ns / 1_000_000_000.0),
                "%.3f" % age_ms,
                "%.3f" % (offset_ns / 1_000_000.0),
                "%.3f" % relative_age_ms,
                inter_arrival_ms,
                message.header.frame_id,
                point_count,
                *measurements,
            )
        )
        self._record_count += 1
        if self._record_count % 128 == 0:
            self._stream.flush()

    def close(self):
        if not self._stream.closed:
            self._stream.flush()
            self._stream.close()


def handle_terminate(unused_signum, unused_frame):
    raise KeyboardInterrupt


def default_output():
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    return Path("log/slam_timing") / ("slam_timing_%s.csv" % timestamp)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=None, help="CSV output path")
    args = parser.parse_args(argv)
    output_path = args.output or default_output()

    rclpy.init(args=None)
    recorder = SlamTimingRecorder(output_path)
    signal.signal(signal.SIGTERM, handle_terminate)
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.close()
        recorder.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
