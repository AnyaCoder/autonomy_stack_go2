#!/usr/bin/env python3
"""Exercise the Wi-Fi command relay in DDS domain 91 on loopback only.

Run with the Go2 SDK/CycloneDDS venv. GO2_RELAY_PATH can select a deployed copy.
Run separately from the other domain 91 tests. No packets are published on a
robot network interface or in its DDS domain.
"""
import importlib.util
import multiprocessing as mp
import os
from pathlib import Path
import time
import unittest

from cyclonedds.domain import Domain, DomainParticipant
from cyclonedds.pub import DataWriter
from cyclonedds.qos import Qos, Policy
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.util import duration
from unitree_sdk2py.idl.unitree_api.msg.dds_ import (
    Request_, RequestHeader_, RequestIdentity_, RequestLease_, RequestPolicy_)

spec = importlib.util.spec_from_file_location(
    'wifi_relay', os.environ.get('GO2_RELAY_PATH',
        str(Path(__file__).resolve().parents[1] / 'tools/native_wifi_relay.py')))
relay = importlib.util.module_from_spec(spec)
# The module contains an IDL dataclass; register it before executing decorators.
import sys
sys.modules[spec.name] = relay
spec.loader.exec_module(relay)


def local_participant(unused_interface):
    config = ('<CycloneDDS><Domain Id="91"><General><Interfaces>'
              '<NetworkInterface name="lo" multicast="true" />'
              '</Interfaces></General></Domain></CycloneDDS>')
    return Domain(91, config), DomainParticipant(91)


def worker(target, stop, requests):
    relay.participant = local_participant
    relay.report = lambda *args, **kwargs: None
    getattr(relay, target)(stop, requests)


class WirelessRequestRelayTest(unittest.TestCase):
    def test_forward_once_preserve_request_and_never_echo_native_requests(self):
        ctx = mp.get_context('spawn')
        stop = ctx.Event()
        requests = ctx.Queue(4)
        children = [ctx.Process(target=worker, args=(name, stop, requests))
                    for name in ('receive_requests', 'transmit_requests')]
        for child in children:
            child.start()
        try:
            domain, node = local_participant('lo')
            qos = Qos(Policy.Reliability.Reliable(duration(milliseconds=100)),
                      Policy.History.KeepLast(16))
            wifi = DataWriter(node, Topic(node, relay.WIFI_REQUEST_TOPIC, Request_), qos)
            native = DataWriter(node, Topic(node, relay.SPORT_REQUEST_TOPIC, Request_), qos)
            reader = DataReader(node, Topic(node, relay.SPORT_REQUEST_TOPIC, Request_), qos)
            # This installation uses the older CycloneDDS Python API without
            # endpoint-match accessors. Allow local discovery before writing.
            time.sleep(3)
            def request(identity, api, parameter):
                return Request_(RequestHeader_(RequestIdentity_(identity, api),
                    RequestLease_(0), RequestPolicy_(0, True)), parameter, [])
            # A request already on the native topic must never enter the relay.
            native.write(request(101, 1003, '{}'))
            wifi.write(request(202, 1008, '{"x":0.1,"y":0,"z":0}'))
            wifi.write(request(203, 1003, '{}'))
            samples = []
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                samples += [m for m in reader.take(32)
                            if getattr(m.sample_info, 'valid_data', False)]
                time.sleep(.02)
            self.assertEqual([101, 202, 203], sorted(m.header.identity.id for m in samples))
            move = next(m for m in samples if m.header.identity.id == 202)
            self.assertEqual(1008, move.header.identity.api_id)
            self.assertEqual('{"x":0.1,"y":0,"z":0}', move.parameter)
            self.assertTrue(all(child.is_alive() for child in children))
        finally:
            stop.set()
            for child in children:
                child.join(4)
                if child.is_alive():
                    child.terminate()
                    child.join(2)
            requests.cancel_join_thread()
            requests.close()


if __name__ == '__main__':
    unittest.main()
