#!/usr/bin/env python3
"""Generate NavSatFix from simulated local odometry for Part 2 testing."""

from __future__ import annotations

import math
import random
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

from .geo_utils import GeoPoint, enu_to_wgs84


class FakeGPSNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_gps_node")
        self.declare_parameter("odom_topic", "/odometry/filtered/local")
        self.declare_parameter("fix_topic", "/fix")
        self.declare_parameter("truth_topic", "/fake_gps/sim_truth")
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("origin_lat", math.nan)
        self.declare_parameter("origin_lon", math.nan)
        self.declare_parameter("origin_alt", 0.0)
        self.declare_parameter("noise_horizontal_std_m", 1.5)
        self.declare_parameter("noise_vertical_std_m", 3.0)
        self.declare_parameter("frame_id", "gps_link")
        self.declare_parameter("random_seed", 4508)
        self.declare_parameter("fault_no_fix_start_sec", -1.0)
        self.declare_parameter("fault_no_fix_duration_sec", 0.0)

        odom_topic = str(self.get_parameter("odom_topic").value)
        fix_topic = str(self.get_parameter("fix_topic").value)
        truth_topic = str(self.get_parameter("truth_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.noise_horizontal_std_m = float(self.get_parameter("noise_horizontal_std_m").value)
        self.noise_vertical_std_m = float(self.get_parameter("noise_vertical_std_m").value)
        self.no_fix_start_sec = float(self.get_parameter("fault_no_fix_start_sec").value)
        self.no_fix_duration_sec = max(0.0, float(self.get_parameter("fault_no_fix_duration_sec").value))

        random_seed = int(self.get_parameter("random_seed").value)
        self.rng = random.Random(random_seed)
        self.boot_sec = self._now_sec()

        self.origin = self._load_origin()
        self.fix_pub = self.create_publisher(NavSatFix, fix_topic, 20)
        self.truth_pub = self.create_publisher(NavSatFix, truth_topic, 20)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 20)
        self.get_logger().info(
            f"fake_gps origin=({self.origin.lat:.8f}, {self.origin.lon:.8f}, {self.origin.alt:.3f}) "
            f"noise(h={self.noise_horizontal_std_m:.2f}m, v={self.noise_vertical_std_m:.2f}m)"
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _load_origin(self) -> GeoPoint:
        lat = float(self.get_parameter("origin_lat").value)
        lon = float(self.get_parameter("origin_lon").value)
        alt = float(self.get_parameter("origin_alt").value)
        if math.isfinite(lat) and math.isfinite(lon):
            return GeoPoint(lat=lat, lon=lon, alt=alt)

        waypoints_file = str(self.get_parameter("waypoints_file").value).strip()
        if not waypoints_file:
            raise RuntimeError("fake_gps_node needs origin_lat/lon or waypoints_file with origin_lat/origin_lon.")
        path = Path(waypoints_file)
        if not path.exists():
            raise RuntimeError(f"waypoints_file does not exist: {path}")
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        if "origin_lat" not in data or "origin_lon" not in data:
            raise RuntimeError("waypoints_file missing origin_lat/origin_lon for fake GPS.")
        return GeoPoint(
            lat=float(data["origin_lat"]),
            lon=float(data["origin_lon"]),
            alt=float(data.get("origin_alt", 0.0)),
        )

    def _in_no_fix_window(self, now_sec: float) -> bool:
        if self.no_fix_start_sec < 0.0 or self.no_fix_duration_sec <= 0.0:
            return False
        elapsed = now_sec - self.boot_sec
        return self.no_fix_start_sec <= elapsed <= (self.no_fix_start_sec + self.no_fix_duration_sec)

    @staticmethod
    def _build_msg(stamp, frame_id: str, geo: GeoPoint, std_h: float, std_v: float, valid: bool) -> NavSatFix:
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.latitude = geo.lat
        msg.longitude = geo.lon
        msg.altitude = geo.alt
        if valid:
            msg.status.status = NavSatStatus.STATUS_FIX
            msg.position_covariance[0] = std_h * std_h
            msg.position_covariance[4] = std_h * std_h
            msg.position_covariance[8] = std_v * std_v
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        else:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        return msg

    def _odom_cb(self, msg: Odometry) -> None:
        now_sec = self._now_sec()
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        z = float(msg.pose.pose.position.z)

        truth_geo = enu_to_wgs84(x, y, z, self.origin)
        truth_msg = self._build_msg(
            stamp=msg.header.stamp,
            frame_id=self.frame_id,
            geo=truth_geo,
            std_h=0.01,
            std_v=0.01,
            valid=True,
        )
        self.truth_pub.publish(truth_msg)

        noisy_x = x + self.rng.gauss(0.0, self.noise_horizontal_std_m)
        noisy_y = y + self.rng.gauss(0.0, self.noise_horizontal_std_m)
        noisy_z = z + self.rng.gauss(0.0, self.noise_vertical_std_m)
        noisy_geo = enu_to_wgs84(noisy_x, noisy_y, noisy_z, self.origin)
        valid = not self._in_no_fix_window(now_sec)
        fix_msg = self._build_msg(
            stamp=msg.header.stamp,
            frame_id=self.frame_id,
            geo=noisy_geo,
            std_h=self.noise_horizontal_std_m,
            std_v=self.noise_vertical_std_m,
            valid=valid,
        )
        self.fix_pub.publish(fix_msg)


def main() -> None:
    rclpy.init()
    node = FakeGPSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
