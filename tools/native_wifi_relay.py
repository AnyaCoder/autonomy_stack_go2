#!/usr/bin/env python3
"""Relay sensors to wlan0 and dedicated Wi-Fi requests to the Sport API."""
import argparse
from collections import OrderedDict
from dataclasses import dataclass
import hashlib
import json
import multiprocessing as mp
import os
from pathlib import Path
import queue
import signal
import time

from cyclonedds.domain import Domain, DomainParticipant
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.annotations import final, autoid
from cyclonedds.idl.types import array, float64
from cyclonedds.pub import DataWriter
from cyclonedds.qos import Qos, Policy
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.util import duration
from unitree_sdk2py.idl.std_msgs.msg.dds_ import Header_
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Quaternion_, Vector3_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_
from unitree_sdk2py.idl.unitree_api.msg.dds_ import Request_


@dataclass
@final
@autoid('sequential')
class Imu_(IdlStruct, typename='sensor_msgs.msg.dds_.Imu_'):
    header: Header_
    orientation: Quaternion_
    orientation_covariance: array[float64, 9]
    angular_velocity: Vector3_
    angular_velocity_covariance: array[float64, 9]
    linear_acceleration: Vector3_
    linear_acceleration_covariance: array[float64, 9]


TOPICS = (
    ('rt/utlidar/imu', Imu_),
    ('rt/utlidar/cloud', PointCloud2_),
    ('rt/sportmodestate', SportModeState_),
    ('rt/lowstate', LowState_),
)
STATUS = Path.home() / '.cache/go2-wifi-relay'
WIFI_REQUEST_TOPIC = 'rt/go2/wifi/sport/request'
SPORT_REQUEST_TOPIC = 'rt/api/sport/request'


def participant(interface):
    peers = ('<Discovery><Peers><Peer Address="192.168.76.2" /></Peers></Discovery>'
             if interface == 'wlan0' else '')
    allow_multicast = 'spdp' if interface == 'wlan0' else 'true'
    config = ('<CycloneDDS><Domain Id="0"><General><Interfaces>'
              '<NetworkInterface name="%s" multicast="true" />'
              '</Interfaces><AllowMulticast>%s</AllowMulticast></General>'
              '%s</Domain></CycloneDDS>') % (interface, allow_multicast, peers)
    domain = Domain(0, config)
    return domain, DomainParticipant(0)


def report(role, counts, extra=None):
    state = {'role': role, 'time': time.time(), 'counts': counts}
    if extra:
        state.update(extra)
    STATUS.mkdir(mode=0o700, parents=True, exist_ok=True)
    tmp = STATUS / (role + '.tmp')
    tmp.write_text(json.dumps(state))
    tmp.replace(STATUS / (role + '.json'))
    print(json.dumps(state), flush=True)


def receive(stop, queues, interface='eth0', seconds=0):
    if queues is not None:
        for item in queues:
            item.cancel_join_thread()
    domain, node = participant(interface)
    qos = Qos(Policy.Reliability.BestEffort, Policy.History.KeepLast(4))
    readers = [DataReader(node, Topic(node, name, cls), qos) for name, cls in TOPICS]
    counts = {name: 0 for name, _ in TOPICS}
    byte_counts = {name: 0 for name, _ in TOPICS}
    queue_drops = {name: 0 for name, _ in TOPICS}
    recent = [OrderedDict() for _ in TOPICS]
    deadline = time.monotonic() + seconds if seconds else float('inf')
    next_report = 0
    while not stop.is_set() and time.monotonic() < deadline:
        for idx, reader in enumerate(readers):
            for sample in reader.take(16):
                if not getattr(sample.sample_info, 'valid_data', False):
                    continue
                payload = sample.serialize()
                if queues is not None:
                    # Suppress repeated samples, including any returning relay
                    # traffic, before crossing the process boundary again.
                    digest = hashlib.blake2b(payload, digest_size=16).digest()
                    seen = recent[idx]
                    if digest in seen:
                        continue
                    seen[digest] = None
                    if len(seen) > 2048:
                        seen.popitem(last=False)
                    try:
                        queues[idx].put_nowait(payload)
                    except queue.Full:
                        try:
                            queues[idx].get_nowait()
                            queue_drops[TOPICS[idx][0]] += 1
                        except queue.Empty:
                            pass
                        try:
                            queues[idx].put_nowait(payload)
                        except queue.Full:
                            queue_drops[TOPICS[idx][0]] += 1
                counts[TOPICS[idx][0]] += 1
                byte_counts[TOPICS[idx][0]] += len(payload)
        if time.monotonic() >= next_report:
            report('probe' if queues is None else 'source', counts, {
                'interface': interface,
                'bytes': byte_counts,
                'queue_drops': queue_drops,
            })
            next_report = time.monotonic() + 5
        stop.wait(.002)
    report('probe' if queues is None else 'source', counts, {
        'interface': interface,
        'bytes': byte_counts,
        'queue_drops': queue_drops,
    })


def transmit(stop, queues, indices, role):
    domain, node = participant('wlan0')
    # Point clouds are large sensor samples.  Reliable delivery lets a lost
    # fragment hold the writer's small history and turns the stream into an
    # ever older backlog on a busy 2.4 GHz link.  Keep only the newest cloud;
    # control/state topics retain reliable delivery.
    reliable_qos = Qos(Policy.Reliability.Reliable(duration(milliseconds=100)),
                        Policy.History.KeepLast(8))
    cloud_qos = Qos(Policy.Reliability.BestEffort,
                    Policy.History.KeepLast(1))
    writers = {
        idx: DataWriter(node, Topic(node, *TOPICS[idx]),
                        cloud_qos if idx == 1 else reliable_qos)
        for idx in indices
    }
    counts = {TOPICS[idx][0]: 0 for idx in indices}
    byte_counts = {TOPICS[idx][0]: 0 for idx in indices}
    next_report = 0
    while not stop.is_set():
        did_work = False
        for idx, writer in writers.items():
            for _ in range(16 if idx != 1 else 1):
                try:
                    payload = queues[idx].get_nowait()
                except queue.Empty:
                    break
                sample = TOPICS[idx][1].deserialize(payload)
                writer.write(sample)
                did_work = True
                counts[TOPICS[idx][0]] += 1
                byte_counts[TOPICS[idx][0]] += len(payload)
        if time.monotonic() >= next_report:
            report(role, counts, {'interface': 'wlan0', 'bytes': byte_counts})
            next_report = time.monotonic() + 5
        stop.wait(.0005 if did_work else .002)


def receive_requests(stop, requests):
    requests.cancel_join_thread()
    domain, node = participant('wlan0')
    qos = Qos(Policy.Reliability.Reliable(duration(milliseconds=100)),
              Policy.History.KeepLast(4))
    # Never subscribe to the native Sport topic on wlan0: the robot's own
    # requests may be visible there and must not be injected back into eth0.
    reader = DataReader(node, Topic(node, WIFI_REQUEST_TOPIC, Request_), qos)
    counts = {'received': 0, 'queue_drops': 0}
    next_report = 0
    while not stop.is_set():
        for sample in reader.take(16):
            if not getattr(sample.sample_info, 'valid_data', False):
                continue
            counts['received'] += 1
            payload = (time.monotonic(), sample.serialize())
            try:
                requests.put_nowait(payload)
            except queue.Full:
                counts['queue_drops'] += 1
                try:
                    requests.get_nowait()
                except queue.Empty:
                    pass
                try:
                    requests.put_nowait(payload)
                except queue.Full:
                    counts['queue_drops'] += 1
        if time.monotonic() >= next_report:
            report('wifi_requests', counts, {'topic': WIFI_REQUEST_TOPIC})
            next_report = time.monotonic() + 5
        stop.wait(.002)


def transmit_requests(stop, requests):
    domain, node = participant('eth0')
    qos = Qos(Policy.Reliability.Reliable(duration(milliseconds=100)),
              Policy.History.KeepLast(4))
    writer = DataWriter(node, Topic(node, SPORT_REQUEST_TOPIC, Request_), qos)
    counts = {'forwarded': 0, 'expired': 0}
    next_report = 0
    while not stop.is_set():
        try:
            received_at, payload = requests.get(timeout=.02)
        except queue.Empty:
            pass
        else:
            if time.monotonic() - received_at > .3:
                counts['expired'] += 1
            else:
                writer.write(Request_.deserialize(payload))
                counts['forwarded'] += 1
        if time.monotonic() >= next_report:
            report('sport_requests', counts, {'topic': SPORT_REQUEST_TOPIC})
            next_report = time.monotonic() + 5


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe', choices=['eth0', 'wlan0'])
    parser.add_argument('--seconds', type=float, default=10)
    args = parser.parse_args()
    ctx = mp.get_context('spawn')
    stop = ctx.Event()
    stopping = [False]
    def on_signal(*_):
        stopping[0] = True
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    if args.probe:
        receive(stop, None, args.probe, args.seconds)
        return
    queues = [ctx.Queue(maxsize=4 if idx == 1 else 32)
              for idx in range(len(TOPICS))]
    requests = ctx.Queue(maxsize=4)
    children = [ctx.Process(target=transmit,
                            args=(stop, queues, (0, 2, 3), 'wifi_imu_state')),
                ctx.Process(target=transmit,
                            args=(stop, queues, (1,), 'wifi_cloud')),
                ctx.Process(target=receive, args=(stop, queues)),
                ctx.Process(target=receive_requests, args=(stop, requests)),
                ctx.Process(target=transmit_requests, args=(stop, requests))]
    try:
        for child in children:
            child.start()
        while not stopping[0]:
            if any(not child.is_alive() for child in children):
                raise RuntimeError('DDS relay worker stopped')
            time.sleep(.2)
    finally:
        stop.set()
        for child in children:
            if child.pid is None:
                continue
            child.join(3)
            if child.is_alive():
                child.terminate()
                child.join(2)
            if child.is_alive():
                child.kill()
                child.join()
        for item in queues + [requests]:
            item.cancel_join_thread()
            item.close()


if __name__ == '__main__':
    main()
