#!/usr/bin/env python3
"""Isolated dynamicinfo forwarding regression; requires ROS 2 and Python zenoh."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import zlib

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from std_msgs.msg import String
import zenoh


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bridge', required=True)
    parser.add_argument('--domain', type=int, default=84)
    parser.add_argument('--port', type=int, default=17448)
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    with tempfile.TemporaryDirectory(prefix='dynamicinfo_test_') as directory:
        os.environ['ROS_LOG_DIR'] = directory
        config = zenoh.Config()
        endpoint = f'tcp/127.0.0.1:{args.port}'
        config.insert_json5('listen/endpoints', f'["{endpoint}"]')
        config.insert_json5('scouting/multicast/enabled', 'false')
        session = zenoh.open(config)
        rclpy.init()
        node = rclpy.create_node('dynamicinfo_verifier')
        topics = ['/dynamicinfo', '/scene/dynamicinfo']
        received = {topic: [] for topic in topics}
        qos = QoSProfile(depth=1024, reliability=ReliabilityPolicy.RELIABLE)
        subscriptions = [node.create_subscription(
            String, topic, lambda raw, t=topic: received[t].append(raw), qos, raw=True)
            for topic in topics]
        publishers = {topic: session.declare_publisher('rt' + topic) for topic in topics}

        def wait_for(predicate, timeout=15):
            deadline = time.monotonic() + timeout
            while not predicate():
                if time.monotonic() >= deadline:
                    raise RuntimeError('Timed out waiting for discovery or forwarded data')
                rclpy.spin_once(node, timeout_sec=0.01)

        log_path = Path(directory) / 'bridge.log'
        with log_path.open('w') as log:
            bridge = subprocess.Popen([
                args.bridge, '--endpoint', endpoint, '--recording-mode',
                '--predeclare-topic', '/dynamicinfo'], stdout=log, stderr=subprocess.STDOUT)
            try:
                wait_for(lambda: node.count_publishers('/dynamicinfo') == 1)
                # The namespaced topic must be created automatically from its first payload.
                bootstrap = b'{}'
                deadline = time.monotonic() + 15
                while not received['/scene/dynamicinfo']:
                    if time.monotonic() >= deadline:
                        raise RuntimeError('Automatic publisher discovery failed')
                    publishers['/scene/dynamicinfo'].put(bootstrap)
                    until = time.monotonic() + 0.1
                    while time.monotonic() < until:
                        rclpy.spin_once(node, timeout_sec=0.01)
                # Flush bootstrap messages before checking exact ordered content.
                until = time.monotonic() + 0.5
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=0.01)
                for values in received.values():
                    values.clear()
                texts = ['{}', '[]',
                         ' {"车辆":{"category":"car","speed":2.5,"velocity_unit":"m/s",'
                         '"velocity":{"x":2.5,"y":0,"z":0},"bbox":{'
                         '"center":{"x":1,"y":2,"z":3},'
                         '"extent":{"x":2,"y":1,"z":1},'
                         '"forward":{"x":1,"y":0,"z":0}}}}\n',
                         '"{\\"nested\\":true}"', '{"cdr":true}']
                expected = [serialize_message(String(data=text)) for text in texts]
                expected = [raw + bytes((-len(raw)) % 4) for raw in expected]
                for index, text in enumerate(texts):
                    payload = expected[index] if index == len(texts) - 1 else text.encode('utf-8')
                    for publisher in publishers.values():
                        publisher.put(payload)
                    wait_for(lambda: all(len(v) >= index + 1 for v in received.values()))
                for topic in topics:
                    assert received[topic] == expected, (topic, received[topic])
                endpoints = node.get_publishers_info_by_topic('/dynamicinfo')
                assert endpoints[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE
                bridge.send_signal(signal.SIGINT)
                assert bridge.wait(timeout=20) == 0
                crc = 0
                for raw in expected:
                    crc = zlib.crc32(raw, crc)
                output = log_path.read_text()
                assert ('created ROS2 publisher: /dynamicinfo [std_msgs/msg/String], '
                        'qos=reliable depth=1024') in output, output
                stats = [line for line in output.splitlines()
                         if 'final_stats' in line and 'key=rt/dynamicinfo ' in line]
                assert len(stats) == 1, output
                assert f'payload_crc32={crc}' in stats[0], stats[0]
                print('PASS: automatic routing, predeclare, JSON/UTF-8 preservation, '
                      'CDR passthrough, recording QoS and CDR CRC32')
            finally:
                if bridge.poll() is None:
                    bridge.send_signal(signal.SIGINT)
                    try:
                        bridge.wait(timeout=20)
                    except subprocess.TimeoutExpired:
                        bridge.kill()
                        bridge.wait()
                node.destroy_node()
                rclpy.shutdown()
                session.close()


if __name__ == '__main__':
    main()
