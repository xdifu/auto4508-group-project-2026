#!/usr/bin/env python3
"""Republish LaserScan with a fresh timestamp for Nav2 consumers."""

from copy import deepcopy

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanSanitizer(Node):
    def __init__(self) -> None:
        super().__init__("scan_sanitizer")
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_nav")
        self.declare_parameter("max_age_sec", 0.75)
        self.declare_parameter("future_tolerance_sec", 0.25)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.max_age = Duration(seconds=float(self.get_parameter("max_age_sec").value))
        self.future_tolerance = Duration(seconds=float(self.get_parameter("future_tolerance_sec").value))

        self.publisher = self.create_publisher(LaserScan, output_topic, 20)
        self.create_subscription(LaserScan, input_topic, self._scan_cb, 20)

    def _scan_cb(self, msg: LaserScan) -> None:
        now = self.get_clock().now()
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        # Nav2 only needs temporally valid scan stamps; use "now" whenever the
        # driver timestamp is too old or too far ahead of the current ROS clock.
        if stamp.nanoseconds <= 0 or stamp < (now - self.max_age) or stamp > (now + self.future_tolerance):
            sanitized = deepcopy(msg)
            sanitized.header.stamp = now.to_msg()
            self.publisher.publish(sanitized)
            return
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = ScanSanitizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
