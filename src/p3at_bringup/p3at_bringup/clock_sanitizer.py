#!/usr/bin/env python3
"""Republish Gazebo clock as a monotonic ROS /clock stream."""

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class ClockSanitizer(Node):
    def __init__(self) -> None:
        super().__init__("clock_sanitizer")
        self.declare_parameter("input_topic", "/clock_raw")
        self.declare_parameter("output_topic", "/clock")
        self.declare_parameter("reset_tolerance_sec", 1.0)

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.reset_tolerance_ns = int(
            self.get_parameter("reset_tolerance_sec").get_parameter_value().double_value * 1_000_000_000
        )

        self.last_ns = -1
        self.dropped_backwards = 0
        self.publisher = self.create_publisher(Clock, output_topic, 100)
        self.create_subscription(Clock, input_topic, self._clock_cb, 100)

    def _clock_cb(self, msg: Clock) -> None:
        stamp_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if self.last_ns >= 0 and stamp_ns + self.reset_tolerance_ns < self.last_ns:
            self.get_logger().warn(
                f"Detected simulator clock reset {self.last_ns} -> {stamp_ns}; accepting new epoch."
            )
            self.last_ns = -1

        if stamp_ns < self.last_ns:
            self.dropped_backwards += 1
            if self.dropped_backwards <= 5:
                self.get_logger().warn(
                    f"Dropping backwards clock sample {self.last_ns} -> {stamp_ns}"
                )
            return

        self.last_ns = stamp_ns
        self.dropped_backwards = 0
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = ClockSanitizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
