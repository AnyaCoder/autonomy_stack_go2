#!/usr/bin/env python3
"""Serve D435i previews on Go2 or publish camera previews on the operator PC."""

import argparse
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import urlopen


def serve_realsense(args):
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    rclpy.init()
    node = Node("go2_realsense_preview")
    frames = {}
    lock = threading.Lock()
    last_encoded = {}

    def receive(kind, message):
        now = time.monotonic()
        if now - last_encoded.get(kind, 0) < 1.0 / args.fps:
            return
        try:
            channels = 3 if kind == "color" else 1
            dtype = np.uint8 if kind == "color" else np.uint16
            row = np.frombuffer(message.data, dtype=dtype).reshape(
                message.height, message.step // dtype().itemsize)
            image = row[:, :message.width * channels].reshape(
                message.height, message.width, channels)
            if kind == "color":
                if message.encoding == "rgb8":
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                elif message.encoding != "bgr8":
                    return
                extension = ".jpg"
                options = [cv2.IMWRITE_JPEG_QUALITY, 75]
            else:
                if message.encoding not in ("16UC1", "mono16"):
                    return
                image = image.reshape(message.height, message.width)
                extension = ".png"
                options = [cv2.IMWRITE_PNG_COMPRESSION, 1]
            ok, encoded = cv2.imencode(extension, image, options)
            if ok:
                with lock:
                    sequence = frames.get(kind, (None, None, 0))[2] + 1
                    frames[kind] = (encoded.tobytes(), time.monotonic(), sequence)
                last_encoded[kind] = now
        except (ValueError, cv2.error) as exc:
            node.get_logger().warning("Invalid {} frame: {}".format(kind, exc))

    node.create_subscription(Image, "/camera/color/image_raw",
                             lambda msg: receive("color", msg), qos_profile_sensor_data)
    node.create_subscription(Image, "/camera/depth/image_rect_raw",
                             lambda msg: receive("depth", msg), qos_profile_sensor_data)

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            kind = {"/color.jpg": "color", "/depth.png": "depth"}.get(self.path)
            if kind is None:
                self.send_error(404)
                return
            with lock:
                frame = frames.get(kind)
            if frame is None or time.monotonic() - frame[1] > 3:
                self.send_error(503, "No fresh camera frame")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg" if kind == "color" else "image/png")
            self.send_header("Content-Length", str(len(frame[0])))
            self.send_header("X-Frame-Sequence", str(frame[2]))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(frame[0])
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *unused):
            pass

    server = ThreadingHTTPServer((args.bind, args.port), Handler)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        server.serve_forever()
    finally:
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()


def publish(args):
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    rclpy.init()
    node = Node("go2_camera_preview_bridge")
    if args.hosts is None:
        interface = os.environ.get("GO2_NETWORK_INTERFACE")
        args.hosts = (["192.168.76.1"] if interface == "wlp4s0" else
                      ["192.168.123.18"] if interface == "enp2s0" else
                      ["192.168.123.18", "192.168.76.1"])
    sources = (
        ("front", 8765, "/frame.jpg", "/go2/front_camera/image_raw"),
        ("color", 8776, "/color.jpg", "/go2/realsense/color/image_raw"),
        ("depth", 8776, "/depth.png", "/go2/realsense/depth/image_raw"),
    )
    publishers = {kind: node.create_publisher(Image, topic, qos_profile_sensor_data)
                  for kind, _, _, topic in sources}
    last_sequence = {}
    last_warning = {}

    def poll(kind, port, path, _topic):
        for host in args.hosts:
            try:
                with urlopen("http://{}:{}{}".format(host, port, path), timeout=0.3) as response:
                    sequence = response.headers.get("X-Frame-Sequence")
                    if last_sequence.get(kind) == (host, sequence):
                        return
                    data = response.read(4 * 1024 * 1024)
                flag = cv2.IMREAD_UNCHANGED if kind == "depth" else cv2.IMREAD_COLOR
                image = cv2.imdecode(np.frombuffer(data, np.uint8), flag)
                if image is None:
                    raise ValueError("image decode failed")
                message = Image()
                message.header.stamp = node.get_clock().now().to_msg()
                message.header.frame_id = "go2_front_camera_optical_frame" if kind == "front" else "camera_{}_optical_frame".format(kind)
                message.height, message.width = image.shape[:2]
                message.encoding = "16UC1" if kind == "depth" else "bgr8"
                message.step = message.width * (2 if kind == "depth" else 3)
                message.data = image.tobytes()
                publishers[kind].publish(message)
                if kind not in last_sequence:
                    node.get_logger().info("{} preview: {}x{} from {}".format(
                        kind, message.width, message.height, host))
                last_sequence[kind] = (host, sequence)
                return
            except Exception as exc:
                error = exc
        if time.monotonic() - last_warning.get(kind, 0) > 10:
            node.get_logger().warning("{} preview unavailable: {}".format(kind, error))
            last_warning[kind] = time.monotonic()

    node.create_timer(1.0 / args.fps, lambda: [poll(*source) for source in sources])
    node.get_logger().info("Camera previews publishing at up to {} Hz".format(args.fps))
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest="mode", required=True)
    server = modes.add_parser("serve-realsense")
    server.add_argument("--bind", default="0.0.0.0")
    server.add_argument("--port", type=int, default=8776)
    server.add_argument("--fps", type=float, default=5)
    client = modes.add_parser("publish")
    client.add_argument("--hosts", nargs="+")
    client.add_argument("--fps", type=float, default=5)
    args, _ = parser.parse_known_args()
    if args.fps <= 0:
        parser.error("fps must be positive")
    (serve_realsense if args.mode == "serve-realsense" else publish)(args)


if __name__ == "__main__":
    main()
