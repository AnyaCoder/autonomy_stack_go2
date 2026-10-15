#!/usr/bin/env python3
"""List, merge, and publish Point-LIO PCD map results."""

import argparse
import os
import re
import shutil
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAPS = ROOT / "maps"
COPY_SIZE = 1024 * 1024


def map_directories():
    if not MAPS.is_dir():
        return []
    return sorted(
        (path for path in MAPS.iterdir() if path.is_dir()),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )


def resolve_session(name):
    directories = map_directories()
    if name in (None, "latest"):
        if not directories:
            raise ValueError(f"没有地图目录: {MAPS}")
        return directories[0]

    candidate = Path(name)
    if not candidate.is_absolute():
        candidate = MAPS / candidate
    candidate = candidate.resolve()
    try:
        candidate.relative_to(MAPS.resolve())
    except ValueError as exc:
        raise ValueError("批次路径必须位于 maps/ 目录内") from exc
    if not candidate.is_dir():
        raise ValueError(f"地图批次不存在: {candidate}")
    return candidate


def point_cloud_files(directory):
    chunks = [
        path for path in directory.glob("scans_*.pcd")
        if re.fullmatch(r"scans_\d+\.pcd", path.name)
    ]
    chunks.sort(key=lambda path: int(re.fullmatch(r"scans_(\d+)\.pcd", path.name).group(1)))
    tail = directory / "scans.pcd"
    files = chunks
    if tail.is_file():
        files.append(tail)
    return files


def list_maps():
    directories = map_directories()
    if not directories:
        print(f"尚无建图结果。结果会保存在 {MAPS}/office_时间戳-随机号/")
        return 0

    print(f"{'批次':42} {'状态':24} {'PCD':>5} {'大小':>10}")
    for directory in directories:
        files = point_cloud_files(directory)
        size = sum(path.stat().st_size for path in files)
        if (directory / "scans.pcd").is_file():
            state = "已结束，尾块已保存"
        elif files:
            state = "运行中或异常中断"
        else:
            state = "运行中，尚无分块"
        print(f"{directory.name:42.42} {state:24} {len(files):5d} {size:10,d} B")
    return 0


def read_pcd_header(path):
    with path.open("rb") as stream:
        lines = []
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"PCD 文件头不完整: {path}")
            lines.append(line.decode("ascii").strip())
            if lines[-1].startswith("DATA "):
                break
        header = {}
        for line in lines:
            parts = line.split()
            if parts:
                header[parts[0].upper()] = parts[1:]
        if header.get("DATA") != ["binary"]:
            raise ValueError(f"仅支持 Point-LIO binary PCD: {path}")
        for key in ("FIELDS", "SIZE", "TYPE", "COUNT", "POINTS"):
            if key not in header:
                raise ValueError(f"PCD 缺少 {key}: {path}")
        fields = header["FIELDS"]
        sizes = [int(value) for value in header["SIZE"]]
        types = header["TYPE"]
        counts = [int(value) for value in header["COUNT"]]
        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise ValueError(f"PCD 字段定义不一致: {path}")
        point_step = sum(size * count for size, count in zip(sizes, counts))
        points = int(header["POINTS"][0])
        payload_offset = stream.tell()
        expected_size = point_step * points
        actual_size = path.stat().st_size - payload_offset
        if actual_size != expected_size:
            raise ValueError(f"PCD 数据长度异常: {path}，预期 {expected_size}，实际 {actual_size}")
        return {
            "fields": fields,
            "sizes": sizes,
            "types": types,
            "counts": counts,
            "points": points,
            "point_step": point_step,
            "payload_offset": payload_offset,
        }


def compatible(first, other, path):
    for key in ("fields", "sizes", "types", "counts", "point_step"):
        if first[key] != other[key]:
            raise ValueError(f"PCD 字段不匹配: {path} ({key})")


def make_pcd_header(info, points):
    return (
        "VERSION .7\n"
        f"FIELDS {' '.join(info['fields'])}\n"
        f"SIZE {' '.join(map(str, info['sizes']))}\n"
        f"TYPE {' '.join(info['types'])}\n"
        f"COUNT {' '.join(map(str, info['counts']))}\n"
        f"WIDTH {points}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {points}\n"
        "DATA binary\n"
    ).encode("ascii")


def merge_session(name):
    directory = resolve_session(name)
    files = point_cloud_files(directory)
    if not files:
        raise ValueError(f"批次中还没有 PCD 分块: {directory}")

    infos = [read_pcd_header(path) for path in files]
    for path, info in zip(files[1:], infos[1:]):
        compatible(infos[0], info, path)
    total_points = sum(info["points"] for info in infos)
    destination = directory / "map.pcd"
    temporary = directory / ".map.pcd.tmp"

    try:
        with temporary.open("wb") as output:
            output.write(make_pcd_header(infos[0], total_points))
            for path, info in zip(files, infos):
                with path.open("rb") as source:
                    source.seek(info["payload_offset"])
                    shutil.copyfileobj(source, output, COPY_SIZE)
        os.replace(temporary, destination)
    except Exception:
        temporary.unlink(missing_ok=True)
        raise

    size = destination.stat().st_size
    print(f"已合并 {len(files)} 个分块、{total_points:,} 点: {destination} ({size:,} B)")
    return destination


def publish_map(path):
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2, PointField
    except ImportError as exc:
        raise RuntimeError("请使用 ./go2.sh map-view；它会加载本机 ROS 2 Foxy 环境") from exc

    info = read_pcd_header(path)
    fields = []
    offset = 0
    datatype = {
        ("F", 4): PointField.FLOAT32,
        ("F", 8): PointField.FLOAT64,
        ("I", 1): PointField.INT8,
        ("I", 2): PointField.INT16,
        ("I", 4): PointField.INT32,
        ("U", 1): PointField.UINT8,
        ("U", 2): PointField.UINT16,
        ("U", 4): PointField.UINT32,
    }
    for name, size, field_type, count in zip(
        info["fields"], info["sizes"], info["types"], info["counts"]
    ):
        try:
            field_datatype = datatype[(field_type, size)]
        except KeyError as exc:
            raise ValueError(f"RViz 不支持 PCD 字段类型: {field_type}{size}") from exc
        fields.append(PointField(name=name, offset=offset, datatype=field_datatype, count=count))
        offset += size * count

    with path.open("rb") as stream:
        stream.seek(info["payload_offset"])
        payload = stream.read()

    message = PointCloud2()
    message.header.frame_id = "camera_init"
    message.height = 1
    message.width = info["points"]
    message.fields = fields
    message.is_bigendian = False
    message.point_step = info["point_step"]
    message.row_step = info["point_step"] * info["points"]
    message.data = payload
    message.is_dense = True

    rclpy.init()
    node = Node("saved_map_publisher")
    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    publisher = node.create_publisher(PointCloud2, "/saved_map", qos)
    remaining = {"count": 0}

    def send_map():
        if publisher.get_subscription_count() == 0:
            return
        publisher.publish(message)
        remaining["count"] += 1
        if remaining["count"] == 1:
            node.get_logger().info(
                f"已发布地图 {info['points']:,} 点，文件 {path}"
            )
        if remaining["count"] >= 1:
            timer.cancel()

    timer = node.create_timer(1.0, send_map)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("list")
    resolve_parser = subparsers.add_parser("resolve")
    resolve_parser.add_argument("session", nargs="?", default="latest")
    merge_parser = subparsers.add_parser("merge")
    merge_parser.add_argument("session", nargs="?", default="latest")
    publish_parser = subparsers.add_parser("publish")
    publish_parser.add_argument("pcd", type=Path)
    args = parser.parse_args()

    try:
        if args.command == "list":
            return list_maps()
        if args.command == "resolve":
            print(resolve_session(args.session))
            return 0
        if args.command == "merge":
            merge_session(args.session)
            return 0
        if args.command == "publish":
            publish_map(args.pcd)
            return 0
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"地图操作失败: {exc}", file=sys.stderr)
        return 1
    return 2


if __name__ == "__main__":
    sys.exit(main())
