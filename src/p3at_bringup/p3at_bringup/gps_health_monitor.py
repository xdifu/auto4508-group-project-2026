#!/usr/bin/env python3
"""GPS health grading node for Part 2 mission gating."""

from __future__ import annotations

import json
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

from .geo_utils import GeoPoint, haversine_distance_m


class GPSHealthMonitor(Node):
    def __init__(self) -> None:
        super().__init__("gps_health_monitor")
        self.declare_parameter("input_topic", "/fix")
        self.declare_parameter("output_topic", "/mission/gps_health")
        self.declare_parameter("health_debug_topic", "/mission/gps_health_detail")
        # Real-field GNSS covariance can momentarily exceed lab thresholds.
        # Keep LOST/FATAL for true outages, but avoid over-triggering FATAL on
        # moderate covariance noise.
        self.declare_parameter("warn_std_m", 4.5)
        self.declare_parameter("max_std_m", 8.0)
        self.declare_parameter("short_loss_seconds", 5.0)
        self.declare_parameter("hard_loss_seconds", 30.0)
        self.declare_parameter("jump_threshold_m", 5.0)
        self.declare_parameter("publish_hz", 5.0)

        self.warn_std_m = float(self.get_parameter("warn_std_m").value)
        self.max_std_m = float(self.get_parameter("max_std_m").value)
        self.short_loss_seconds = float(self.get_parameter("short_loss_seconds").value)
        self.hard_loss_seconds = float(self.get_parameter("hard_loss_seconds").value)
        self.jump_threshold_m = float(self.get_parameter("jump_threshold_m").value)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        detail_topic = str(self.get_parameter("health_debug_topic").value)
        publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))

        self.health_pub = self.create_publisher(String, output_topic, 10)
        self.detail_pub = self.create_publisher(String, detail_topic, 10)
        self.create_subscription(NavSatFix, input_topic, self._fix_cb, 20)
        self.timer = self.create_timer(1.0 / publish_hz, self._publish_status)

        self.boot_sec = self._now_sec()
        self.last_fix_msg_sec: Optional[float] = None
        self.last_valid_fix_sec: Optional[float] = None
        self.last_valid_point: Optional[GeoPoint] = None
        self.current_std_m = math.inf
        self.force_fatal_until_sec = 0.0
        self.last_status = ""

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _is_covariance_known(msg: NavSatFix) -> bool:
        return msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN

    @staticmethod
    def _horizontal_std_m(msg: NavSatFix) -> float:
        h_var = max(float(msg.position_covariance[0]), float(msg.position_covariance[4]), 0.0)
        return math.sqrt(h_var)

    @staticmethod
    def _is_fix_valid(msg: NavSatFix) -> bool:
        return msg.status.status >= NavSatStatus.STATUS_FIX and GPSHealthMonitor._is_covariance_known(msg)

    def _fix_cb(self, msg: NavSatFix) -> None:
        now_sec = self._now_sec()
        self.last_fix_msg_sec = now_sec

        if not self._is_fix_valid(msg):
            return

        std_m = self._horizontal_std_m(msg)
        if not math.isfinite(std_m):
            return
        self.current_std_m = std_m
        self.last_valid_fix_sec = now_sec
        current_point = GeoPoint(lat=msg.latitude, lon=msg.longitude, alt=msg.altitude)

        if self.last_valid_point is not None:
            jump_m = haversine_distance_m(self.last_valid_point, current_point)
            dynamic_threshold = max(3.0 * max(std_m, 0.01), self.jump_threshold_m)
            if jump_m > dynamic_threshold:
                self.force_fatal_until_sec = max(self.force_fatal_until_sec, now_sec + 2.0)
                self.get_logger().error(
                    f"GPS jump detected ({jump_m:.2f}m > {dynamic_threshold:.2f}m). Forcing FATAL state."
                )
        self.last_valid_point = current_point

    def _derive_status(self) -> str:
        now_sec = self._now_sec()
        if now_sec < self.force_fatal_until_sec:
            return "FATAL"
        if self.last_valid_fix_sec is None:
            if now_sec - self.boot_sec <= self.short_loss_seconds:
                return "DEGRADED"
            if now_sec - self.boot_sec <= self.hard_loss_seconds:
                return "LOST"
            return "FATAL"

        gap = now_sec - self.last_valid_fix_sec
        if self.current_std_m > self.max_std_m:
            return "FATAL"
        if gap > self.hard_loss_seconds:
            return "FATAL"
        if gap >= self.short_loss_seconds:
            return "LOST"
        if self.current_std_m > self.warn_std_m:
            return "DEGRADED"
        return "HEALTHY"

    def _publish_status(self) -> None:
        status = self._derive_status()
        msg = String()
        msg.data = status
        self.health_pub.publish(msg)

        detail = String()
        detail.data = json.dumps(
            {
                "status": status,
                "std_m": None if not math.isfinite(self.current_std_m) else round(self.current_std_m, 4),
                "last_valid_age_s": None
                if self.last_valid_fix_sec is None
                else round(self._now_sec() - self.last_valid_fix_sec, 3),
            },
            ensure_ascii=False,
        )
        self.detail_pub.publish(detail)

        if status != self.last_status:
            self.last_status = status
            self.get_logger().info(f"GPS health -> {status}")


def main() -> None:
    rclpy.init()
    node = GPSHealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
