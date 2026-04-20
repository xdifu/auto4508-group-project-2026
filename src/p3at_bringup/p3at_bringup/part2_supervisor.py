#!/usr/bin/env python3
"""Real-world Part 2 supervisor for waypoint navigation, inspection, and summary orchestration."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SaveMap
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, NavSatFix, NavSatStatus
from std_msgs.msg import Bool, String
from tf_transformations import euler_from_quaternion

from .arrival_judge import ArrivalJudge
from .geo_utils import GeoPoint
from .goal_builder import GoalOffset, build_pose_stamped, compute_goal_xyyaw
from .waypoint_loader import MissionPlan, Waypoint, WaypointLoader


class Part2Supervisor(Node):
    def __init__(self) -> None:
        super().__init__("part2_supervisor")

        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("policy_file", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("autostart", True)
        self.declare_parameter("require_safety_topics", True)
        self.declare_parameter("run_id", "part2_run")
        self.declare_parameter("artifacts_root", str(Path.cwd() / "artifacts" / "part2_runs" / "part2_run"))
        self.declare_parameter("map_save_service", "/map_saver/save_map")
        self.declare_parameter("loop_hz", 10.0)
        self.declare_parameter("state_pub_hz", 2.0)
        self.declare_parameter("goal_accept_timeout_sec", 3.0)
        self.declare_parameter("goal_execution_timeout_sec", 90.0)
        self.declare_parameter("nav_retry_budget", 2)
        self.declare_parameter("inspection_retry_budget", 1)
        self.declare_parameter("pass_side_retry_budget", 1)
        self.declare_parameter("arrival_min_standoff_m", 1.0)
        self.declare_parameter("arrival_target_standoff_m", 1.4)
        self.declare_parameter("weave_midpoint_retry_budget", 2)
        self.declare_parameter("search_arc_degrees", 120.0)
        self.declare_parameter("search_arc_radius_extra", 0.6)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.autostart = bool(self.get_parameter("autostart").value)
        self.require_safety_topics = bool(self.get_parameter("require_safety_topics").value)
        self.run_id = str(self.get_parameter("run_id").value).strip() or "part2_run"
        self.artifacts_root = Path(str(self.get_parameter("artifacts_root").value)).expanduser()
        if not self.artifacts_root.is_absolute():
            self.artifacts_root = Path.cwd() / self.artifacts_root
        self.map_dir = self.artifacts_root / "map"
        self.map_dir.mkdir(parents=True, exist_ok=True)
        self.goal_accept_timeout_sec = float(self.get_parameter("goal_accept_timeout_sec").value)
        self.goal_execution_timeout_sec = float(self.get_parameter("goal_execution_timeout_sec").value)
        self.nav_retry_budget = int(self.get_parameter("nav_retry_budget").value)
        self.inspection_retry_budget = int(self.get_parameter("inspection_retry_budget").value)
        self.pass_side_retry_budget = int(self.get_parameter("pass_side_retry_budget").value)
        self.arrival_min_standoff_m = float(self.get_parameter("arrival_min_standoff_m").value)
        self.arrival_target_standoff_m = float(self.get_parameter("arrival_target_standoff_m").value)
        self.weave_midpoint_retry_budget = int(self.get_parameter("weave_midpoint_retry_budget").value)
        self.search_arc_degrees = float(self.get_parameter("search_arc_degrees").value)
        self.search_arc_radius_extra = float(self.get_parameter("search_arc_radius_extra").value)
        self.map_save_service = str(self.get_parameter("map_save_service").value)

        waypoints_file = str(self.get_parameter("waypoints_file").value).strip()
        policy_file = str(self.get_parameter("policy_file").value).strip()
        if not waypoints_file or not policy_file:
            raise RuntimeError("Both `waypoints_file` and `policy_file` are required.")

        self.loader = WaypointLoader(waypoints_file=waypoints_file, policy_file=policy_file)
        self.plan: Optional[MissionPlan] = None
        self.state = "INIT"
        self.prev_state = "INIT"

        self.gps_health = "DEGRADED"
        self.gps_std_m = math.inf
        self.last_fix_point: Optional[GeoPoint] = None
        self.global_odom_ready = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_speed = 0.0
        self.latest_scan: Optional[LaserScan] = None

        self.automated_enabled = self.autostart and not self.require_safety_topics
        self.deadman_pressed = self.autostart and not self.require_safety_topics
        self.estop = False

        self.current_idx = 0
        self.current_sequence: List[PoseStamped] = []
        self.current_sequence_kind = "approach"
        self.current_sequence_goal_index = 0
        self.current_goal_sent = False
        self.current_goal_handle = None
        self.current_goal_token: Optional[int] = None
        self.canceled_goal_token: Optional[int] = None
        self.goal_request_seq = 0
        self.goal_send_future = None
        self.goal_result_future = None
        self.goal_sent_at_sec = 0.0
        self.goal_cancel_requested = False
        self.goal_failed = False
        self.goal_succeeded = False
        self.goal_status_code = GoalStatus.STATUS_UNKNOWN

        self.nav_retry_count = 0
        self.inspection_retry_count = 0
        self.pass_side_retry_count = 0
        self.inspection_attempt_idx = 1
        self.current_weave_retry_count = 0

        self.awaiting_vision = False
        self.vision_result: Optional[Dict] = None
        self.vision_request_key: Optional[str] = None

        self.arrival_judge = ArrivalJudge(confirm_frames=3, pass_side_ratio=0.7)
        self.action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.map_save_client = self.create_client(SaveMap, self.map_save_service)
        self.map_save_future = None
        self.map_save_requested = False
        self.map_save_done = False

        self.state_pub = self.create_publisher(String, "/mission/state", 10)
        self.event_pub = self.create_publisher(String, "/mission/event", 20)
        self.nav_pub = self.create_publisher(String, "/mission/navigation", 20)

        self.create_subscription(String, "/mission/gps_health", self._gps_health_cb, 20)
        self.create_subscription(NavSatFix, "/fix", self._fix_cb, 20)
        self.create_subscription(Odometry, "/odometry/filtered/global", self._global_odom_cb, 20)
        self.create_subscription(Bool, "/safety/automated_enabled", self._automated_cb, 20)
        self.create_subscription(Bool, "/safety/deadman_pressed", self._deadman_cb, 20)
        self.create_subscription(Bool, "/safety/estop", self._estop_cb, 20)
        self.create_subscription(String, "/mission/vision", self._vision_cb, 20)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 20)

        self.loop_timer = self.create_timer(1.0 / max(2.0, float(self.get_parameter("loop_hz").value)), self._loop)
        self.heartbeat_timer = self.create_timer(
            1.0 / max(0.5, float(self.get_parameter("state_pub_hz").value)),
            self._publish_state,
        )

        self.get_logger().info(f"Part 2 supervisor initialized. run_id={self.run_id}")

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _publish_state(self) -> None:
        self.state_pub.publish(String(data=self.state))

    def _publish_event(self, event_type: str, data=None) -> None:
        payload = {
            "run_id": self.run_id,
            "ts": round(self._now_sec(), 3),
            "type": event_type,
            "state": self.state,
            "data": data if data is not None else {},
        }
        self.event_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _set_state(self, new_state: str) -> None:
        if self.state == new_state:
            return
        self.prev_state = self.state
        self.state = new_state
        self._publish_event("state_change", {"from": self.prev_state, "to": new_state})
        self.get_logger().info(f"State -> {new_state}")

    def _gps_health_cb(self, msg: String) -> None:
        self.gps_health = msg.data.strip().upper() if msg.data else "DEGRADED"

    def _fix_cb(self, msg: NavSatFix) -> None:
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        self.last_fix_point = GeoPoint(lat=msg.latitude, lon=msg.longitude, alt=msg.altitude)
        if msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            h_var = max(float(msg.position_covariance[0]), float(msg.position_covariance[4]), 0.0)
            self.gps_std_m = math.sqrt(h_var)

    def _global_odom_cb(self, msg: Odometry) -> None:
        self.global_odom_ready = True
        self.robot_x = float(msg.pose.pose.position.x)
        self.robot_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        _, _, self.robot_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        vx = float(msg.twist.twist.linear.x)
        vz = float(msg.twist.twist.angular.z)
        self.robot_speed = math.hypot(vx, vz)

    def _automated_cb(self, msg: Bool) -> None:
        self.automated_enabled = bool(msg.data)

    def _deadman_cb(self, msg: Bool) -> None:
        self.deadman_pressed = bool(msg.data)

    def _estop_cb(self, msg: Bool) -> None:
        self.estop = bool(msg.data)

    def _vision_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return
        wp_id = str(payload.get("wp_id", ""))
        if wp_id != str(self._public_wp_id()):
            return
        attempt_idx = int(payload.get("attempt_idx", self.inspection_attempt_idx))
        if attempt_idx != self.inspection_attempt_idx:
            return
        self.vision_result = payload
        self.vision_request_key = f"{wp_id}:{attempt_idx}"

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _is_safety_ready(self) -> bool:
        if self.estop:
            return False
        if not self.require_safety_topics:
            return self.autostart
        return self.automated_enabled and self.deadman_pressed

    def _ensure_plan(self) -> bool:
        if self.plan is not None:
            return True
        datum = self.loader.yaml_datum if self.loader.yaml_datum is not None else self.last_fix_point
        if datum is None:
            return False
        fallback_home = None
        if self.loader.yaml_home is None and self.loader.return_home:
            fallback_home = self.last_fix_point
            if fallback_home is None:
                return False
        self.plan = self.loader.build_plan(datum=datum, fallback_home=fallback_home)
        self._publish_event("plan_loaded", {"mission_id": self.plan.mission_id, "count": len(self.plan.waypoints)})
        return True

    def _current_waypoint(self) -> Optional[Waypoint]:
        if self.plan is None or self.current_idx >= len(self.plan.waypoints):
            return None
        return self.plan.waypoints[self.current_idx]

    def _public_wp_id(self) -> int:
        wp = self._current_waypoint()
        if wp is None:
            return -1
        if wp.is_home_return:
            return 0
        return self.current_idx + 1

    def _current_kind(self) -> str:
        wp = self._current_waypoint()
        if wp is not None and wp.is_home_return:
            return "home_return"
        return "waypoint"

    def _nav_payload(self, phase: str, status: str, reason: str = "", pass_side_verified: bool = False) -> Dict:
        wp = self._current_waypoint()
        if wp is None:
            return {}
        return {
            "run_id": self.run_id,
            "wp_id": self._public_wp_id(),
            "kind": self._current_kind(),
            "phase": phase,
            "status": status,
            "attempt_idx": self.inspection_attempt_idx,
            "arrival_radius_m": round(float(wp.arrival_radius), 3),
            "pose_map": {"x": round(float(wp.x), 3), "y": round(float(wp.y), 3), "yaw": round(self._heading_for_waypoint(wp), 3)},
            "pose_gps": {"lat": float(wp.lat), "lon": float(wp.lon)},
            "reason": reason,
            "pass_side_verified": pass_side_verified,
            "ts": round(self._now_sec(), 3),
        }

    def _publish_nav(self, phase: str, status: str, reason: str = "", pass_side_verified: bool = False) -> None:
        payload = self._nav_payload(phase, status, reason, pass_side_verified)
        if payload:
            self.nav_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _heading_for_waypoint(self, wp: Waypoint) -> float:
        if self.plan is None:
            return math.atan2(wp.y - self.robot_y, wp.x - self.robot_x)
        if self.current_idx + 1 < len(self.plan.waypoints):
            nxt = self.plan.waypoints[self.current_idx + 1]
            return math.atan2(nxt.y - wp.y, nxt.x - wp.x)
        if self.current_idx > 0:
            prev = self.plan.waypoints[self.current_idx - 1]
            return math.atan2(wp.y - prev.y, wp.x - prev.x)
        return math.atan2(wp.y - self.robot_y, wp.x - self.robot_x)

    def _build_tangent_poses(self, wp: Waypoint) -> List[PoseStamped]:
        policy = self.plan.policies[wp.policy]
        lateral = policy.goal_offset_lateral if abs(policy.goal_offset_lateral) > 1e-6 else max(1.0, wp.arrival_radius)
        longitudinal = policy.goal_offset_longitudinal if abs(policy.goal_offset_longitudinal) > 1e-6 else max(1.0, wp.arrival_radius)
        offset = GoalOffset(lateral=lateral, longitudinal=longitudinal)
        heading = self._heading_for_waypoint(wp)
        tx, ty, tyaw = compute_goal_xyyaw(
            cone_x=wp.x,
            cone_y=wp.y,
            theta_in=heading,
            pass_side=wp.pass_side,
            offset=offset,
        )
        pre_dist = 2.5
        px = tx - pre_dist * math.cos(heading)
        py = ty - pre_dist * math.sin(heading)
        stamp = self.get_clock().now().to_msg()
        return [
            build_pose_stamped(self.frame_id, stamp, px, py, tyaw),
            build_pose_stamped(self.frame_id, stamp, tx, ty, tyaw),
        ]

    def _build_backoff_pose(self, wp: Waypoint) -> PoseStamped:
        dx = self.robot_x - wp.x
        dy = self.robot_y - wp.y
        norm = math.hypot(dx, dy)
        if norm < 1e-3:
            heading = self._heading_for_waypoint(wp)
            dx = -math.cos(heading)
            dy = -math.sin(heading)
            norm = 1.0
        scale = self.arrival_target_standoff_m / norm
        gx = wp.x + dx * scale
        gy = wp.y + dy * scale
        gyaw = self._heading_for_waypoint(wp)
        return build_pose_stamped(self.frame_id, self.get_clock().now().to_msg(), gx, gy, gyaw)

    def _clusters_from_scan(self) -> List[List[tuple[float, float]]]:
        if self.latest_scan is None:
            return []
        pts = []
        angle = self.latest_scan.angle_min
        for rng in self.latest_scan.ranges:
            if math.isfinite(rng) and 0.2 < rng < 6.0:
                x = rng * math.cos(angle)
                y = rng * math.sin(angle)
                if x > 0.2:
                    pts.append((x, y))
                else:
                    pts.append(None)
            else:
                pts.append(None)
            angle += self.latest_scan.angle_increment

        clusters: List[List[tuple[float, float]]] = []
        current: List[tuple[float, float]] = []
        prev = None
        for pt in pts:
            if pt is None:
                if current:
                    clusters.append(current)
                    current = []
                    prev = None
                continue
            if prev is None or math.hypot(pt[0] - prev[0], pt[1] - prev[1]) < 0.35:
                current.append(pt)
            else:
                if current:
                    clusters.append(current)
                current = [pt]
            prev = pt
        if current:
            clusters.append(current)
        return clusters

    def _build_weave_midpoints(self) -> List[PoseStamped]:
        wp = self._current_waypoint()
        if wp is None:
            return []
        clusters = []
        for cluster in self._clusters_from_scan():
            if len(cluster) < 3:
                continue
            xs = [p[0] for p in cluster]
            ys = [p[1] for p in cluster]
            if max(xs) - min(xs) > 0.8 or max(ys) - min(ys) > 0.8:
                continue
            clusters.append((sum(xs) / len(xs), sum(ys) / len(ys)))

        left = sorted([c for c in clusters if c[1] > 0.0], key=lambda p: p[0])
        right = sorted([c for c in clusters if c[1] < 0.0], key=lambda p: p[0])
        if not left and not right:
            return []

        corridor_half_width = 0.8
        midpoints = []
        for i in range(max(len(left), len(right))):
            l_pt = left[i] if i < len(left) else None
            r_pt = right[i] if i < len(right) else None
            if l_pt is not None and r_pt is not None:
                mx = 0.5 * (l_pt[0] + r_pt[0])
                my = 0.5 * (l_pt[1] + r_pt[1])
            elif l_pt is not None:
                mx = l_pt[0]
                my = l_pt[1] - corridor_half_width
            elif r_pt is not None:
                mx = r_pt[0]
                my = r_pt[1] + corridor_half_width
            else:
                continue
            map_x = self.robot_x + mx * math.cos(self.robot_yaw) - my * math.sin(self.robot_yaw)
            map_y = self.robot_y + mx * math.sin(self.robot_yaw) + my * math.cos(self.robot_yaw)
            midpoints.append((map_x, map_y))

        poses: List[PoseStamped] = []
        for idx, (mx, my) in enumerate(midpoints[:4]):
            if idx + 1 < len(midpoints):
                nx, ny = midpoints[idx + 1]
                yaw = math.atan2(ny - my, nx - mx)
            else:
                yaw = self._heading_for_waypoint(wp)
            poses.append(build_pose_stamped(self.frame_id, self.get_clock().now().to_msg(), mx, my, yaw))
        return poses

    def _build_search_arc(self, wp: Waypoint) -> List[PoseStamped]:
        radius = min(2.4, max(1.6, wp.arrival_radius + self.search_arc_radius_extra))
        start_angle = math.atan2(self.robot_y - wp.y, self.robot_x - wp.x)
        total = math.radians(self.search_arc_degrees)
        step = total / 3.0
        poses = []
        for idx in range(1, 4):
            angle = start_angle - idx * step
            gx = wp.x + radius * math.cos(angle)
            gy = wp.y + radius * math.sin(angle)
            gyaw = angle - math.pi / 2.0
            poses.append(build_pose_stamped(self.frame_id, self.get_clock().now().to_msg(), gx, gy, gyaw))
        return poses

    def _start_sequence(self, poses: List[PoseStamped], sequence_kind: str) -> bool:
        if not poses:
            return False
        self.current_sequence = poses
        self.current_sequence_kind = sequence_kind
        self.current_sequence_goal_index = 0
        self.current_goal_sent = False
        self.goal_failed = False
        self.goal_succeeded = False
        self.goal_status_code = GoalStatus.STATUS_UNKNOWN
        self.current_goal_handle = None
        self.current_goal_token = None
        return self._dispatch_next_goal()

    def _dispatch_next_goal(self) -> bool:
        if self.current_sequence_goal_index >= len(self.current_sequence):
            return False
        if not self.action_client.wait_for_server(timeout_sec=0.1):
            return False
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.current_sequence[self.current_sequence_goal_index]
        self.goal_sent_at_sec = self._now_sec()
        self.current_goal_sent = True
        self.goal_request_seq += 1
        goal_token = self.goal_request_seq
        self.current_goal_token = goal_token
        self.goal_send_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_cb)
        self.goal_send_future.add_done_callback(lambda future, token=goal_token: self._goal_response_cb(future, token))
        return True

    def _feedback_cb(self, _msg) -> None:
        return

    def _goal_response_cb(self, future, goal_token: int) -> None:
        if goal_token != self.current_goal_token:
            return
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.goal_failed = True
            self.current_goal_sent = False
            self.current_goal_token = None
            self._publish_event("goal_send_failed", {"error": str(exc)})
            return
        if not goal_handle.accepted:
            self.goal_failed = True
            self.current_goal_sent = False
            self.current_goal_token = None
            self._publish_event("goal_rejected", {"index": self.current_sequence_goal_index})
            return
        self.current_goal_handle = goal_handle
        self.goal_result_future = goal_handle.get_result_async()
        self.goal_result_future.add_done_callback(lambda future, token=goal_token: self._goal_result_cb(future, token))

    def _goal_result_cb(self, future, goal_token: int) -> None:
        is_current_goal = goal_token == self.current_goal_token
        is_canceled_goal = goal_token == self.canceled_goal_token
        if not is_current_goal and not is_canceled_goal:
            return
        if is_current_goal:
            self.current_goal_sent = False
        try:
            wrapped = future.result()
        except Exception as exc:  # noqa: BLE001
            if is_canceled_goal:
                self.goal_cancel_requested = False
                self.canceled_goal_token = None
                return
            self.goal_failed = True
            self.current_goal_token = None
            self._publish_event("goal_result_failed", {"error": str(exc)})
            return
        if is_current_goal:
            self.goal_status_code = int(wrapped.status)
        status_code = int(wrapped.status)
        if status_code == GoalStatus.STATUS_SUCCEEDED and is_current_goal:
            self.goal_succeeded = True
        elif status_code == GoalStatus.STATUS_CANCELED and is_canceled_goal:
            self.goal_cancel_requested = False
            self.canceled_goal_token = None
        elif is_current_goal:
            self.goal_failed = True

    def _cancel_goal(self) -> None:
        if self.current_goal_handle is None:
            return
        self.goal_cancel_requested = True
        self.canceled_goal_token = self.current_goal_token
        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(lambda _f: None)
        self.current_goal_handle = None
        self.current_goal_token = None

    def _distance_to_waypoint(self) -> Optional[float]:
        wp = self._current_waypoint()
        if wp is None:
            return None
        return math.hypot(self.robot_x - wp.x, self.robot_y - wp.y)

    def _advance_waypoint(self) -> None:
        self.current_idx += 1
        self.nav_retry_count = 0
        self.inspection_retry_count = 0
        self.pass_side_retry_count = 0
        self.inspection_attempt_idx = 1
        self.current_weave_retry_count = 0
        self.vision_result = None
        self.awaiting_vision = False
        self.current_sequence = []
        self.current_sequence_goal_index = 0
        self.current_sequence_kind = "approach"
        self.arrival_judge.reset()

    def _request_map_save(self) -> None:
        if self.map_save_requested:
            return
        if not self.map_save_client.wait_for_service(timeout_sec=0.5):
            self._publish_event("map_save_unavailable", {"service": self.map_save_service})
            self.map_save_done = True
            return
        request = SaveMap.Request()
        request.map_topic = "/map"
        request.map_url = str(self.map_dir / "map")
        request.image_format = "pgm"
        request.map_mode = "trinary"
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65
        self.map_save_requested = True
        self.map_save_future = self.map_save_client.call_async(request)
        self._publish_event("map_save_requested", {"target": request.map_url})

    def _handle_navigation_failure(self, reason: str) -> None:
        wp = self._current_waypoint()
        if wp is None:
            self._set_state("ABORTED")
            return
        if self.nav_retry_count < self.nav_retry_budget and not wp.is_home_return:
            self.nav_retry_count += 1
            self._publish_event("nav_retry", {"retry": self.nav_retry_count, "reason": reason})
            self.current_sequence = []
            self.current_sequence_goal_index = 0
            self.goal_failed = False
            self.goal_succeeded = False
            self._set_state("PLAN_SEGMENT")
            return
        self._publish_nav("navigate", "failed", reason=reason)
        if wp.is_home_return:
            self._set_state("ABORTED")
            return
        self._advance_waypoint()
        self._set_state("PLAN_SEGMENT")

    def _start_inspection(self) -> None:
        dist = self._distance_to_waypoint()
        wp = self._current_waypoint()
        if wp is None or dist is None:
            self._set_state("ABORTED")
            return
        if dist < self.arrival_min_standoff_m:
            backoff = self._build_backoff_pose(wp)
            if self._start_sequence([backoff], "backoff"):
                self._set_state("NAVIGATING")
            else:
                self.goal_failed = True
                self._publish_event("goal_dispatch_failed", {"index": 0, "sequence_kind": "backoff"})
                self._handle_navigation_failure("backoff_goal_dispatch_failed")
            return
        self.awaiting_vision = True
        self.vision_result = None
        self._publish_nav("inspect", "reached")
        self._set_state("INSPECTING")

    def _plan_current_segment(self) -> bool:
        wp = self._current_waypoint()
        if wp is None:
            self._set_state("SAVE_MAP")
            return False
        goals: List[PoseStamped] = []
        sequence_kind = "approach"
        if wp.policy == "weave_through_cones" and not wp.is_home_return and self._public_wp_id() == 2:
            weave_goals = self._build_weave_midpoints()
            if weave_goals:
                sequence_kind = "weave"
                goals.extend(weave_goals)
        goals.extend(self._build_tangent_poses(wp))
        if not goals:
            return False
        self._publish_nav("navigate", "searching")
        return self._start_sequence(goals, sequence_kind)

    def _handle_sequence_completion(self) -> None:
        wp = self._current_waypoint()
        if wp is None:
            self._set_state("SAVE_MAP")
            return
        if self.current_sequence_kind == "search_arc":
            self.awaiting_vision = True
            self.vision_result = None
            self._publish_nav("inspect", "reached")
            self._set_state("INSPECTING")
            return
        if wp.is_home_return:
            self._publish_nav("return_home", "reached", pass_side_verified=True)
            self._advance_waypoint()
            self._set_state("PLAN_SEGMENT")
            return
        self._start_inspection()

    def _loop(self) -> None:
        if self.state == "COMPLETED":
            return
        if self.state == "ABORTED":
            return

        if self.estop and self.state != "SAFE_STOP":
            self._cancel_goal()
            self._set_state("SAFE_STOP")
            return

        if self.state == "INIT":
            self._set_state("WAITING_FOR_GPS")
            return

        if self.state == "WAITING_FOR_GPS":
            if not self._ensure_plan():
                return
            if self.gps_health != "HEALTHY" or not self.global_odom_ready:
                return
            self._set_state("WAITING_FOR_SAFETY")
            return

        if self.state == "WAITING_FOR_SAFETY":
            if self._is_safety_ready():
                self._set_state("PLAN_SEGMENT")
            return

        if self.state == "SAFE_STOP":
            if self.gps_health == "FATAL":
                return
            if self._is_safety_ready():
                self._set_state("PLAN_SEGMENT")
            return

        if self.state == "PLAN_SEGMENT":
            if self.current_idx >= len(self.plan.waypoints):
                self._set_state("SAVE_MAP")
                return
            if not self._plan_current_segment():
                return
            self._set_state("NAVIGATING")
            return

        if self.state == "NAVIGATING":
            wp = self._current_waypoint()
            if wp is None:
                self._set_state("SAVE_MAP")
                return
            if self.current_goal_sent and (self._now_sec() - self.goal_sent_at_sec) > self.goal_execution_timeout_sec:
                self._cancel_goal()
                self.goal_failed = True
                self._publish_event("goal_timeout", {"timeout_sec": self.goal_execution_timeout_sec})
            if self.current_sequence_goal_index == len(self.current_sequence) - 1:
                arrived, dist = self.arrival_judge.update(
                    now_sec=self._now_sec(),
                    robot_x=self.robot_x,
                    robot_y=self.robot_y,
                    robot_yaw=self.robot_yaw,
                    linear_speed_mps=self.robot_speed,
                    target_x=wp.x,
                    target_y=wp.y,
                    arrival_radius=wp.arrival_radius,
                    gps_horizontal_std=self.gps_std_m,
                    nav2_goal_reached=self.goal_succeeded,
                    vision_override=False,
                )
                if arrived:
                    self._cancel_goal()
                    self.goal_succeeded = True
                    self._publish_event("arrival_confirmed", {"distance_m": round(dist, 3)})
            if self.goal_failed:
                self.goal_failed = False
                self._handle_navigation_failure("nav_goal_failed")
                return
            if self.goal_succeeded:
                self.goal_succeeded = False
                self.current_goal_handle = None
                self.current_goal_token = None
                self.current_sequence_goal_index += 1
                if self.current_sequence_goal_index < len(self.current_sequence):
                    if not self._dispatch_next_goal():
                        self.goal_failed = True
                        self._publish_event(
                            "goal_dispatch_failed",
                            {"index": self.current_sequence_goal_index, "sequence_kind": self.current_sequence_kind},
                        )
                else:
                    self._handle_sequence_completion()
            return

        if self.state == "INSPECTING":
            if not self.awaiting_vision:
                self._set_state("PLAN_SEGMENT")
                return
            if self.vision_result is None:
                return
            vision_status = str(self.vision_result.get("status", "failed"))
            pass_side_ok = self.arrival_judge.verify_pass_side(
                self._current_waypoint().x,
                self._current_waypoint().y,
                self._current_waypoint().pass_side,
            )
            if vision_status == "ok" and pass_side_ok:
                self._publish_nav("verified", "reached", pass_side_verified=True)
                self._advance_waypoint()
                self._set_state("PLAN_SEGMENT")
                return
            if vision_status == "ok" and not pass_side_ok and self.pass_side_retry_count < self.pass_side_retry_budget:
                self.pass_side_retry_count += 1
                self.inspection_attempt_idx += 1
                self.vision_result = None
                self.current_sequence = []
                self._publish_event("pass_side_retry", {"retry": self.pass_side_retry_count})
                self._set_state("PLAN_SEGMENT")
                return
            if vision_status in {"partial", "failed"} and self.inspection_retry_count < self.inspection_retry_budget:
                arc = self._build_search_arc(self._current_waypoint())
                self.inspection_retry_count += 1
                self.inspection_attempt_idx += 1
                self.vision_result = None
                self.awaiting_vision = False
                if self._start_sequence(arc, "search_arc"):
                    self._set_state("NAVIGATING")
                    return
            self._publish_nav("inspect", "failed", reason=str(self.vision_result.get("error_reason", "inspection_failed")))
            self._advance_waypoint()
            self._set_state("PLAN_SEGMENT")
            return

        if self.state == "SAVE_MAP":
            if not self.map_save_requested and not self.map_save_done:
                self._request_map_save()
                if self.map_save_done:
                    self._set_state("COMPLETED")
                return
            if self.map_save_future is None:
                self._set_state("COMPLETED")
                return
            if self.map_save_future.done():
                try:
                    response = self.map_save_future.result()
                    self._publish_event("map_save_complete", {"result": bool(response.result)})
                except Exception as exc:  # noqa: BLE001
                    self._publish_event("map_save_failed", {"error": str(exc)})
                self.map_save_done = True
                self._set_state("COMPLETED")


def main() -> None:
    rclpy.init()
    node = Part2Supervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
