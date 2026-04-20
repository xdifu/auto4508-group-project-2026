#!/usr/bin/env python3
"""Republish simulated joint states with monotonic ROS time stamps."""

from rclpy.node import Node
from rclpy.time import Time
import rclpy
from sensor_msgs.msg import JointState


class JointStateSanitizer(Node):
    def __init__(self) -> None:
        super().__init__("joint_state_sanitizer")
        self.declare_parameter("input_topic", "/joint_states_raw")
        self.declare_parameter("output_topic", "/joint_states")
        self.declare_parameter("reset_tolerance_sec", 1.0)

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.reset_tolerance_ns = int(
            self.get_parameter("reset_tolerance_sec").get_parameter_value().double_value * 1_000_000_000
        )

        self.last_stamp_ns = 0
        self.publisher = self.create_publisher(JointState, output_topic, 50)
        self.create_subscription(JointState, input_topic, self._joint_cb, 50)

    def _joint_cb(self, msg: JointState) -> None:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns <= 0:
            stamp_ns = self.get_clock().now().nanoseconds
        if stamp_ns + self.reset_tolerance_ns < self.last_stamp_ns:
            self.get_logger().warn(
                "Detected joint state timestamp reset; accepting new sequence.",
                throttle_duration_sec=5.0,
            )
            self.last_stamp_ns = 0
        if stamp_ns <= self.last_stamp_ns:
            stamp_ns = self.last_stamp_ns + 1_000_000
        self.last_stamp_ns = stamp_ns

        sanitized = JointState()
        sanitized.header = msg.header
        sanitized.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
        sanitized.name = list(msg.name)
        sanitized.position = list(msg.position)
        sanitized.velocity = list(msg.velocity)
        sanitized.effort = list(msg.effort)

        self.publisher.publish(sanitized)


def main() -> None:
    rclpy.init()
    node = JointStateSanitizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        if "Unable to convert call argument" not in str(exc):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
