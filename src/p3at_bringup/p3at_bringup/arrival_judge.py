#!/usr/bin/env python3
"""Arrival and pass-side checks for Part 2 waypoint handling."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple


@dataclass
class PoseSample:
    t_sec: float
    x: float
    y: float
    yaw: float


class ArrivalJudge:
    def __init__(
        self,
        confirm_frames: int = 3,
        speed_guard_mps: float = 0.15,
        speed_guard_duration_sec: float = 1.5,
        pass_side_window_sec: float = 2.0,
        pass_side_ratio: float = 0.8,
    ) -> None:
        self.confirm_frames = max(1, int(confirm_frames))
        self.speed_guard_mps = max(0.0, float(speed_guard_mps))
        self.speed_guard_duration_sec = max(0.0, float(speed_guard_duration_sec))
        self.pass_side_window_sec = max(0.1, float(pass_side_window_sec))
        self.pass_side_ratio = min(1.0, max(0.0, float(pass_side_ratio)))
        self._inside_count = 0
        self._low_speed_since: Optional[float] = None
        self._samples: Deque[PoseSample] = deque()

    def reset(self) -> None:
        self._inside_count = 0
        self._low_speed_since = None
        self._samples.clear()

    @staticmethod
    def distance_m(robot_x: float, robot_y: float, target_x: float, target_y: float) -> float:
        return math.hypot(robot_x - target_x, robot_y - target_y)

    def _track_speed_guard(self, linear_speed_mps: float, now_sec: float) -> bool:
        if linear_speed_mps < self.speed_guard_mps:
            if self._low_speed_since is None:
                self._low_speed_since = now_sec
            return now_sec - self._low_speed_since >= self.speed_guard_duration_sec
        self._low_speed_since = None
        return False

    def update(
        self,
        now_sec: float,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        linear_speed_mps: float,
        target_x: float,
        target_y: float,
        arrival_radius: float,
        gps_horizontal_std: float,
        nav2_goal_reached: bool = False,
        vision_override: bool = False,
    ) -> Tuple[bool, float]:
        """Returns (arrived, distance_to_waypoint_m)."""
        self._samples.append(PoseSample(now_sec, robot_x, robot_y, robot_yaw))
        while self._samples and now_sec - self._samples[0].t_sec > self.pass_side_window_sec:
            self._samples.popleft()

        dist = self.distance_m(robot_x, robot_y, target_x, target_y)
        speed_guard_ok = self._track_speed_guard(linear_speed_mps, now_sec)
        guard_required = gps_horizontal_std >= 0.8 * arrival_radius
        guard_ok = vision_override or (nav2_goal_reached and speed_guard_ok)

        in_radius = dist <= arrival_radius and (not guard_required or guard_ok)
        if in_radius:
            self._inside_count += 1
        else:
            self._inside_count = 0
        return self._inside_count >= self.confirm_frames, dist

    def verify_pass_side(self, cone_x: float, cone_y: float, expected_side: str) -> bool:
        if expected_side == "none":
            return True
        samples = list(self._samples)
        if not samples:
            return False
        ok = 0
        for sample in samples:
            cross = (cone_x - sample.x) * math.sin(sample.yaw) - (cone_y - sample.y) * math.cos(sample.yaw)
            if expected_side == "right" and cross < 0.0:
                ok += 1
            elif expected_side == "left" and cross > 0.0:
                ok += 1
        return (ok / len(samples)) >= self.pass_side_ratio
