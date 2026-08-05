#!/usr/bin/env python3

import argparse
from collections import deque
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def main():
    parser = argparse.ArgumentParser(
        description="Measure PointCloud2 frequency with a reliable DDS subscription")
    parser.add_argument("topic", nargs="?", default="/front_lidar")
    parser.add_argument("--window", type=int, default=1000)
    args = parser.parse_args()

    rclpy.init()
    node = Node("reliable_lidar_hz")
    intervals = deque(maxlen=max(2, args.window))
    last_message = None
    last_print = time.monotonic()

    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )

    def callback(_message):
        nonlocal last_message
        now = time.monotonic()
        if last_message is not None:
            intervals.append(now - last_message)
        last_message = now

    subscription = node.create_subscription(PointCloud2, args.topic, callback, qos, raw=True)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now - last_print < 1.0 or not intervals:
                continue
            values = list(intervals)
            mean = sum(values) / len(values)
            stddev = math.sqrt(sum((value - mean) ** 2 for value in values) / len(values))
            print(
                f"average rate: {1.0 / mean:.3f}\n"
                f"\tmin: {min(values):.3f}s max: {max(values):.3f}s "
                f"std dev: {stddev:.5f}s window: {len(values)}",
                flush=True,
            )
            last_print = now
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
