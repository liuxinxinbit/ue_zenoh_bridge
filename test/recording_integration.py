#!/usr/bin/env python3
"""Isolated Zenoh -> bridge -> reliable ROS subscriber + SQLite bag regression.

Requires sourced ROS Humble and zenoh Python. Creates only the requested
output directory; does not connect to the simulator. Run with an unused ROS domain.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2, PointField, Imu
import zenoh


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bridge', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--domain', type=int, default=83)
    parser.add_argument('--port', type=int, default=17447)
    parser.add_argument('--frames', type=int, default=200)
    parser.add_argument('--burst', type=int, default=20)
    parser.add_argument('--payload-mib', type=int, default=1)
    parser.add_argument('--pause-recorder-frames', type=int, default=0,
                        help='Pause recorder for this many 10 Hz source frames during the run')
    args = parser.parse_args()
    if args.frames < 2 or args.payload_mib < 1 or args.burst < 0:
        parser.error('frames >= 2, payload-mib >= 1 and burst >= 0 are required')
    if not 0 <= args.pause_recorder_frames < args.frames // 2:
        parser.error('pause-recorder-frames must be >= 0 and < half of frames')
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = str(output / 'ros_logs')
    key_prefix = 'bridge_recording_test'
    topics = ['/front_lidar', '/rear_lidar']
    qos_file = Path(__file__).resolve().parents[1] / 'config/lidar_record_qos.yaml'
    endpoint = f'tcp/127.0.0.1:{args.port}'
    config = zenoh.Config()
    config.insert_json5('mode', '"peer"')
    config.insert_json5('listen/endpoints', json.dumps([endpoint]))
    config.insert_json5('scouting/multicast/enabled', 'false')
    session = zenoh.open(config)
    pubs = {topic: session.declare_publisher(
        key_prefix + topic, congestion_control=zenoh.CongestionControl.BLOCK)
        for topic in topics}
    imu_pub = session.declare_publisher(key_prefix + '/front_lidar/imu')
    rclpy.init()
    node = rclpy.create_node('bridge_recording_test')
    received = {topic: [] for topic in topics}
    receive_times = {topic: [] for topic in topics}
    qos = QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE)
    subscriptions = []
    def on_cloud(raw, topic):
        received[topic].append(hashlib.sha256(raw).hexdigest())
        receive_times[topic].append(time.monotonic())

    for topic in topics:
        subscriptions.append(node.create_subscription(
            PointCloud2, topic,
            lambda raw, topic=topic: on_cloud(raw, topic),
            qos, raw=True))

    def spin_until(predicate, seconds, failure):
        deadline = time.monotonic() + seconds
        while not predicate():
            if time.monotonic() >= deadline:
                raise RuntimeError(failure)
            rclpy.spin_once(node, timeout_sec=0.01)

    bridge = bag = None
    logs = []
    try:
        bridge_log = open(output / 'bridge.log', 'w')
        logs.append(bridge_log)
        command = [args.bridge, '--endpoint', endpoint, '--key-expr', key_prefix + '/**',
                   '--strip-prefix', key_prefix, '--async-publish']
        for topic in topics:
            command += ['--predeclare-topic', topic + ':=sensor_msgs/msg/PointCloud2']
        bridge = subprocess.Popen(command, stdout=bridge_log, stderr=subprocess.STDOUT)
        bag_log = open(output / 'recorder.log', 'w')
        logs.append(bag_log)
        bag = subprocess.Popen(
            ['ros2', 'bag', 'record', '-s', 'sqlite3', '-o', str(output / 'bag'),
             '--qos-profile-overrides-path', str(qos_file),
             '--max-cache-size', '104857600', *topics],
            stdout=bag_log, stderr=subprocess.STDOUT, start_new_session=True)
        spin_until(lambda: all(node.count_publishers(t) >= 1 and
                   node.count_subscribers(t) >= 2 for t in topics), 20,
                   'ROS discovery did not find bridge + recorder + verifier')
        # Graph discovery precedes endpoint matching. Allow both transports to settle.
        settle = time.monotonic() + 2
        while time.monotonic() < settle:
            rclpy.spin_once(node, timeout_sec=0.01)

        size = args.payload_mib * 1024 * 1024
        cloud = PointCloud2(height=1, width=size // 16, point_step=16,
                            row_step=size, is_dense=True)
        cloud.header.frame_id = 'integration_lidar'
        cloud.fields = [PointField(name=name, offset=offset, datatype=7, count=1)
                        for name, offset in [('x', 0), ('y', 4), ('z', 8), ('intensity', 12)]]
        cloud.data = bytes(size)
        expected = []
        imu = Imu()
        imu.header.frame_id = 'integration_imu'
        imu_raw = serialize_message(imu)
        begin = time.monotonic()
        next_imu = begin
        for sequence in range(args.frames + args.burst):
            if args.pause_recorder_frames:
                if sequence == args.frames // 3:
                    os.killpg(bag.pid, signal.SIGSTOP)
                elif sequence == args.frames // 3 + args.pause_recorder_frames:
                    os.killpg(bag.pid, signal.SIGCONT)
            if sequence < args.frames:
                deadline = begin + sequence / 10
                while time.monotonic() < deadline:
                    now = time.monotonic()
                    if now >= next_imu:
                        imu_pub.put(imu_raw)
                        next_imu = now + 0.002
                    rclpy.spin_once(node, timeout_sec=0.001)
            cloud.header.stamp.sec = 1000 + sequence // 10
            cloud.header.stamp.nanosec = (sequence % 10) * 100_000_000
            raw = serialize_message(cloud)
            expected.append(hashlib.sha256(raw).hexdigest())
            for pub in pubs.values():
                pub.put(raw)
            rclpy.spin_once(node, timeout_sec=0)
            if sequence == args.frames - 1:
                steady_elapsed = time.monotonic() - begin
        total = args.frames + args.burst
        spin_until(lambda: all(len(v) >= total for v in received.values()), 20,
                   f'reliable subscriber missing frames: {[len(v) for v in received.values()]}')
        bridge.send_signal(signal.SIGINT)
        code = bridge.wait(timeout=20)
        if code != 0:
            raise RuntimeError(f'bridge exited {code}; inspect bridge.log')
        # Keep recorder alive through bridge drain and DDS acknowledgements.
        os.killpg(bag.pid, signal.SIGINT)
        if bag.wait(timeout=20) != 0:
            raise RuntimeError('recorder failed; inspect recorder.log')
        bag_hashes = {topic: [] for topic in topics}
        bag_times = {topic: [] for topic in topics}
        for db in sorted((output / 'bag').glob('*.db3')):
            with sqlite3.connect(f'file:{db}?mode=ro', uri=True) as conn:
                for name, data, stamp in conn.execute(
                    'SELECT topics.name, messages.data, messages.timestamp FROM messages '
                    'JOIN topics ON messages.topic_id=topics.id ORDER BY messages.id'):
                    bag_hashes[name].append(hashlib.sha256(data).hexdigest())
                    bag_times[name].append(stamp / 1e9)
        for topic in topics:
            if received[topic] != expected or bag_hashes[topic] != expected:
                raise RuntimeError(f'frame loss, duplicate, reordering or payload corruption: {topic}')
        report = dict(result='PASS', topics=topics, frames_per_topic=total,
                      steady_frames=args.frames, steady_source_hz=(args.frames - 1) / steady_elapsed,
                      burst_frames=args.burst, payload_bytes=size,
                      recorder_pause_seconds=args.pause_recorder_frames / 10,
                      reliable_counts={k: len(v) for k, v in received.items()},
                      bag_counts={k: len(v) for k, v in bag_hashes.items()},
                      steady_reliable_hz={k: (args.frames - 1) / (v[args.frames - 1] - v[0])
                                          for k, v in receive_times.items()},
                      steady_bag_hz={k: (args.frames - 1) / (v[args.frames - 1] - v[0])
                                    for k, v in bag_times.items()},
                      verification='ordered SHA256 equality for every serialized frame')
        (output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2), flush=True)
    finally:
        if bridge is not None and bridge.poll() is None:
            bridge.send_signal(signal.SIGINT)
            try:
                bridge.wait(timeout=20)
            except subprocess.TimeoutExpired:
                bridge.kill()
                bridge.wait()
        if bag is not None and bag.poll() is None:
            os.killpg(bag.pid, signal.SIGCONT)
            os.killpg(bag.pid, signal.SIGINT)
            try:
                bag.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(bag.pid, signal.SIGKILL)
                bag.wait()
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
        session.close()


if __name__ == '__main__':
    main()
