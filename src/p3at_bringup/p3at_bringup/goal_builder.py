#!/usr/bin/env python3
"""Build Nav2 goals from waypoint geometry and pass-side constraints."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler


@dataclass(frozen=True)
class GoalOffset:
    lateral: float
    longitudinal: float


def side_sign(pass_side: str) -> int:
    if pass_side == "right":
        return 1
    if pass_side == "left":
        return -1
    return 0


def compute_goal_xyyaw(
    cone_x: float,
    cone_y: float,
    theta_in: float,
    pass_side: str,
    offset: GoalOffset,
) -> Tuple[float, float, float]:
    """Compute goal pose using the corrected pass-side formula from the design."""
    sign = side_sign(pass_side)
    lateral = offset.lateral if sign != 0 else 0.0
    longitudinal = offset.longitudinal
    gx = cone_x + longitudinal * math.cos(theta_in) - lateral * math.sin(theta_in) * sign
    gy = cone_y + longitudinal * math.sin(theta_in) + lateral * math.cos(theta_in) * sign
    gyaw = theta_in
    return gx, gy, gyaw


def build_pose_stamped(
    frame_id: str,
    stamp,
    x: float,
    y: float,
    yaw: float,
) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    q = quaternion_from_euler(0.0, 0.0, yaw)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose
