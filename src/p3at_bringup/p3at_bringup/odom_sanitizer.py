#!/usr/bin/env python3
"""Republish simulated odometry with sane covariance defaults and stable TF output."""

import math
from typing import List

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def _default_pose_covariance() -> List[float]:
    cov = [0.0] * 36
    cov[0] = 0.05
    cov[7] = 0.05
    cov[14] = 1e6
    cov[21] = 1e6
    cov[28] = 1e6
    cov[35] = 0.03
    return cov


def _default_twist_covariance() -> List[float]:
    cov = [0.0] * 36
    cov[0] = 0.05
    cov[7] = 0.2
    cov[14] = 1e6
    cov[21] = 1e6
    cov[28] = 1e6
    cov[35] = 0.03
    return cov


class OdomSanitizer(Node):
    def __init__(self) -> None:
        super().__init__("odom_sanitizer")
        self.declare_parameter("input_topic", "/odom")
        self.declare_parameter("output_topic", "/odom_sanitized")
        self.declare_parameter("filtered_topic", "/odometry/filtered")
        self.declare_parameter("publish_filtered", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_link_frame", "base_link")
        self.declare_parameter("reset_tolerance_sec", 1.0)

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        filtered_topic = self.get_parameter("filtered_topic").get_parameter_value().string_value
        self.publish_filtered = self.get_parameter("publish_filtered").get_parameter_value().bool_value
        self.publish_tf = self.get_parameter("publish_tf").get_parameter_value().bool_value
        self.odom_frame = self.get_parameter("odom_frame").get_parameter_value().string_value
        self.base_link_frame = self.get_parameter("base_link_frame").get_parameter_value().string_value
        self.reset_tolerance_ns = int(
            self.get_parameter("reset_tolerance_sec").get_parameter_value().double_value * 1_000_000_000
        )

        self.pose_covariance = _default_pose_covariance()
        self.twist_covariance = _default_twist_covariance()
        self.publisher = self.create_publisher(Odometry, output_topic, 20)
        self.filtered_publisher = self.create_publisher(Odometry, filtered_topic, 20) if self.publish_filtered else None
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.last_stamp_ns = None
        self.create_subscription(Odometry, input_topic, self._odom_cb, 20)

    @staticmethod
    def _needs_covariance_fix(values: List[float]) -> bool:
        return all(abs(v) < 1e-12 for v in values) or any(not math.isfinite(v) for v in values)

    def _odom_cb(self, msg: Odometry) -> None:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self.last_stamp_ns is not None and stamp_ns + self.reset_tolerance_ns < self.last_stamp_ns:
            self.get_logger().warn(
                "Detected odometry timestamp reset; accepting new sequence.",
                throttle_duration_sec=5.0,
            )
            self.last_stamp_ns = None
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            return
        self.last_stamp_ns = stamp_ns

        orientation = msg.pose.pose.orientation
        quat_norm = math.sqrt(
            orientation.x * orientation.x +
            orientation.y * orientation.y +
            orientation.z * orientation.z +
            orientation.w * orientation.w
        )
        values = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z,
        ]
        if quat_norm < 1e-9 or any(not math.isfinite(v) for v in values):
            self.get_logger().warning("Dropping invalid odometry sample.", throttle_duration_sec=5.0)
            return

        if self._needs_covariance_fix(list(msg.pose.covariance)):
            msg.pose.covariance = self.pose_covariance
        if self._needs_covariance_fix(list(msg.twist.covariance)):
            msg.twist.covariance = self.twist_covariance
        if not msg.header.frame_id:
            msg.header.frame_id = self.odom_frame
        if not msg.child_frame_id:
            msg.child_frame_id = self.base_link_frame

        self.publisher.publish(msg)
        if self.filtered_publisher is not None:
            self.filtered_publisher.publish(msg)
        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = msg.header.stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_link_frame
            transform.transform.translation.x = msg.pose.pose.position.x
            transform.transform.translation.y = msg.pose.pose.position.y
            transform.transform.translation.z = msg.pose.pose.position.z
            transform.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = OdomSanitizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
