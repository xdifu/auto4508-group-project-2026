#!/usr/bin/env python3
"""Record the driven path from filtered odometry and export it to CSV."""

import csv
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from tf_transformations import euler_from_quaternion


class PathRecorder(Node):
    def __init__(self) -> None:
        super().__init__("path_recorder")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("path_topic", "/driven_path")
        self.declare_parameter("sample_period", 0.5)
        self.declare_parameter("min_translation", 0.05)
        self.declare_parameter("min_rotation", 0.05)
        self.declare_parameter("max_path_points", 2000)
        self.declare_parameter("output_csv", str(Path.cwd() / "part1_driven_path.csv"))

        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.path_topic = self.get_parameter("path_topic").get_parameter_value().string_value
        self.sample_period = self.get_parameter("sample_period").get_parameter_value().double_value
        self.min_translation = self.get_parameter("min_translation").get_parameter_value().double_value
        self.min_rotation = self.get_parameter("min_rotation").get_parameter_value().double_value
        self.max_path_points = self.get_parameter("max_path_points").get_parameter_value().integer_value
        self.output_csv = Path(self.get_parameter("output_csv").get_parameter_value().string_value)

        self.latest_odom: Optional[Odometry] = None
        self.last_recorded_pose: Optional[PoseStamped] = None
        self.path_msg = PathMsg()
        self.path_pub = self.create_publisher(PathMsg, self.path_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 20)
        self.create_timer(self.sample_period, self._sample)
        self.output_csv.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.output_csv.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp", "x", "y", "z", "roll", "pitch", "yaw"])

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        return ((a - b + 3.141592653589793) % (2.0 * 3.141592653589793)) - 3.141592653589793

    def _should_append(self, pose: PoseStamped) -> bool:
        if self.last_recorded_pose is None:
            return True

        last = self.last_recorded_pose.pose
        dx = pose.pose.position.x - last.position.x
        dy = pose.pose.position.y - last.position.y
        distance = (dx * dx + dy * dy) ** 0.5

        quat = pose.pose.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        last_quat = last.orientation
        _, _, last_yaw = euler_from_quaternion([last_quat.x, last_quat.y, last_quat.z, last_quat.w])
        yaw_delta = abs(self._angle_diff(yaw, last_yaw))

        return distance >= self.min_translation or yaw_delta >= self.min_rotation

    def _sample(self) -> None:
        if self.latest_odom is None:
            return

        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header = self.latest_odom.header
        pose.header.stamp = stamp
        pose.pose = self.latest_odom.pose.pose

        if not self._should_append(pose):
            return

        if not self.path_msg.header.frame_id:
            self.path_msg.header.frame_id = self.latest_odom.header.frame_id
        self.path_msg.header.stamp = stamp
        self.path_msg.poses.append(pose)
        if len(self.path_msg.poses) > self.max_path_points:
            self.path_msg.poses.pop(0)
        self.path_pub.publish(self.path_msg)
        self.last_recorded_pose = pose

        quat = pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.csv_writer.writerow([
            stamp.sec + stamp.nanosec * 1e-9,
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            roll,
            pitch,
            yaw,
        ])
        self.csv_file.flush()

    def destroy_node(self):
        try:
            self.csv_file.close()
        finally:
            return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
