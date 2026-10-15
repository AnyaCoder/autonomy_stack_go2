#!/usr/bin/env python3
"""Run a ROS command in its own process group and reap leftover nodes."""

import os
import signal
import subprocess
import sys
import time


def group_alive(pgid):
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False


def signal_group(pgid, sig):
    try:
        os.killpg(pgid, sig)
    except ProcessLookupError:
        pass


def wait_for_group(pgid, seconds):
    deadline = time.monotonic() + seconds
    while group_alive(pgid) and time.monotonic() < deadline:
        time.sleep(0.1)


def stop_existing():
    markers = (
        "mapping_utlidar.launch", "system_real_robot.launch",
        "system_real_robot_with_route_planner.launch",
    )
    nodes = (
        "/point_lio_unilidar/pointlio_mapping", "/transform_sensors/transform_everything",
        "/transform_sensors/level_map", "/calibrate_imu/calibrate_imu",
        "/terrain_analysis/terrainAnalysis", "/terrain_analysis_ext/terrainAnalysisExt",
        "/local_planner/localPlanner", "/local_planner/pathFollower",
        "/far_planner/far_planner", "/graph_decoder/graph_decoder",
    )
    orphan_markers = (
        "point_lio_unilidar/rviz_cfg/loam_livox2.rviz",
        "far_planner/rviz/default.rviz",
        "__node:=ps3_joy", "__node:=vehicleTransPublisher",
        "__node:=sensorTransPublisher", "__node:=loamInterfaceTransPubVehicle",
    )
    managers = set()
    launches = {}
    groups = set()
    own_group = os.getpgrp()
    for entry in os.scandir("/proc"):
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            with open(entry.path + "/cmdline", "rb") as stream:
                args = [part.decode(errors="replace") for part in stream.read().split(b"\0") if part]
            if not args:
                continue
            managed = any(arg.endswith("/tools/ros_process_group.py") for arg in args)
            launch = "ros2" in " ".join(args) and "launch" in args and any(
                marker in args for marker in markers
            )
            node = (any(marker in arg for arg in args[:2] for marker in nodes) or
                    any(marker in arg for arg in args for marker in orphan_markers))
            group = os.getpgid(int(entry.name))
            if group == own_group:
                continue
            if managed:
                managers.add(int(entry.name))
            elif launch:
                launches[group] = int(entry.name)
                groups.add(group)
            elif node:
                groups.add(group)
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue

    if managers or groups:
        print("Stopping previous Go2 ROS session(s)", flush=True)
    for pid in managers:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + 20
    while managers and time.monotonic() < deadline:
        managers = {pid for pid in managers if os.path.exists("/proc/{}/cmdline".format(pid))}
        time.sleep(0.1)

    for group, pid in launches.items():
        if group_alive(group):
            try:
                os.kill(pid, signal.SIGINT)
            except ProcessLookupError:
                pass
    for group in groups:
        if group not in launches and group_alive(group):
            signal_group(group, signal.SIGINT)
        wait_for_group(group, 5)
    for sig, timeout in ((signal.SIGTERM, 3), (signal.SIGKILL, 0)):
        for group in groups:
            if group_alive(group):
                signal_group(group, sig)
                wait_for_group(group, timeout)


def main():
    args = sys.argv[1:]
    replace = args[:1] == ["--replace"]
    if replace:
        args = args[1:]
    if not args:
        print("usage: ros_process_group.py [--replace] COMMAND [ARG ...]", file=sys.stderr)
        return 2

    if replace:
        stop_existing()

    child = subprocess.Popen(args, start_new_session=True)
    pgid = child.pid
    received = None

    def stop(sig, unused_frame):
        nonlocal received
        if received is None:
            received = sig
            try:
                child.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        deadline = None
        stop_stage = 0
        while child.poll() is None:
            if received is not None and deadline is None and stop_stage == 0:
                deadline = time.monotonic() + 20
            if deadline is not None and time.monotonic() >= deadline:
                if stop_stage == 0:
                    signal_group(pgid, signal.SIGTERM)
                    deadline = time.monotonic() + 5
                    stop_stage = 1
                else:
                    signal_group(pgid, signal.SIGKILL)
                    deadline = None
                    stop_stage = 2
            time.sleep(0.1)
    finally:
        if group_alive(pgid):
            signal_group(pgid, signal.SIGTERM)
            wait_for_group(pgid, 4)
        if group_alive(pgid):
            signal_group(pgid, signal.SIGKILL)
        child.wait()

    if received is not None:
        return 128 + received
    return child.returncode


if __name__ == "__main__":
    sys.exit(main())
