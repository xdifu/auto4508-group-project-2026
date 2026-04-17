#!/usr/bin/env python3
"""Part 2 mission orchestrator for GPS waypoint navigation."""

from __future__ import annotations

import json
import math
from dataclasses import asdict
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, String
from tf_transformations import euler_from_quaternion

from .arrival_judge import ArrivalJudge
from .geo_utils import GeoPoint
from .goal_builder import GoalOffset, build_pose_stamped, compute_goal_xyyaw
from .waypoint_loader import MissionPlan, WaypointLoader


class MissionOrchestrator(Node):
    def __init__(self) -> None:
        super().__init__("mission_orchestrator")
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("policy_file", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("autostart", True)
        self.declare_parameter("require_safety_topics", False)
        self.declare_parameter("waypoint_leave_buffer_m", 0.5)
        self.declare_parameter("photo_timeout_sec", 5.0)
        self.declare_parameter("max_segment_distance_m", 200.0)
        self.declare_parameter("state_pub_hz", 1.0)
        self.declare_parameter("loop_hz", 10.0)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.autostart = bool(self.get_parameter("autostart").value)
        self.require_safety_topics = bool(self.get_parameter("require_safety_topics").value)
        self.leave_buffer_m = float(self.get_parameter("waypoint_leave_buffer_m").value)
        self.photo_timeout_sec = float(self.get_parameter("photo_timeout_sec").value)

        waypoints_file = str(self.get_parameter("waypoints_file").value).strip()
        policy_file = str(self.get_parameter("policy_file").value).strip()
        if not waypoints_file:
            raise RuntimeError("Parameter `waypoints_file` is required.")
        if not policy_file:
            raise RuntimeError("Parameter `policy_file` is required.")

        self.loader = WaypointLoader(
            waypoints_file=waypoints_file,
            policy_file=policy_file,
            max_segment_distance_m=float(self.get_parameter("max_segment_distance_m").value),
        )

        self.plan: Optional[MissionPlan] = None
        self.current_idx = 0
        self.current_goal_sent = False
        self.current_goal_handle = None
        self.current_goal_send_future = None
        self.nav_goal_reached = False
        self.nav_failed = False
        self.nav_retry_count = 0

        self.state = "INIT"
        self.prev_state: Optional[str] = None
        self.state_enter_sec = self._now_sec()
        self.completed_announced = False

        self.gps_health = "DEGRADED"
        self.gps_std_m = math.inf
        self.last_fix_point: Optional[GeoPoint] = None
        self.global_odom_ready = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_speed = 0.0

        self.automated_enabled = self.autostart and not self.require_safety_topics
        self.deadman_pressed = self.autostart and not self.require_safety_topics
        self.estop = False

        self.photo_done_for_waypoint = False
        self.photo_deadline_sec = 0.0

        self.arrival_judge = ArrivalJudge()
        self.action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.state_pub = self.create_publisher(String, "/mission/state", 10)
        self.event_pub = self.create_publisher(String, "/mission/event", 20)
        self.current_wp_pub = self.create_publisher(String, "/mission/current_waypoint", 10)
        self.photo_request_pub = self.create_publisher(String, "/perception/marker_photo_request", 10)

        self.create_subscription(String, "/mission/gps_health", self._gps_health_cb, 20)
        self.create_subscription(NavSatFix, "/fix", self._fix_cb, 20)
        self.create_subscription(Odometry, "/odometry/filtered/global", self._global_odom_cb, 20)
        self.create_subscription(Bool, "/safety/automated_enabled", self._automated_cb, 20)
        self.create_subscription(Bool, "/safety/deadman_pressed", self._deadman_cb, 20)
        self.create_subscription(Bool, "/safety/estop", self._estop_cb, 20)
        self.create_subscription(String, "/perception/marker_photo_done", self._photo_done_cb, 20)

        self.loop_timer = self.create_timer(1.0 / max(2.0, float(self.get_parameter("loop_hz").value)), self._loop)
        self.heartbeat_timer = self.create_timer(
            1.0 / max(0.2, float(self.get_parameter("state_pub_hz").value)),
            self._publish_state,
        )

        self.get_logger().info("Mission orchestrator initialized.")

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _set_state(self, new_state: str) -> None:
        if new_state == self.state:
            return
        self.prev_state = self.state
        self.state = new_state
        self.state_enter_sec = self._now_sec()
        self._publish_event("state_change", {"from": self.prev_state, "to": self.state})
        self.get_logger().info(f"State -> {self.state}")

    def _publish_state(self) -> None:
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def _publish_event(self, event_type: str, data=None, waypoint_id: Optional[str] = None) -> None:
        payload = {
            "t": round(self._now_sec(), 3),
            "type": event_type,
            "waypoint_id": waypoint_id,
            "data": data if data is not None else {},
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.event_pub.publish(msg)

    @staticmethod
    def _parse_photo_done(payload: str) -> dict:
        try:
            parsed = json.loads(payload)
            if isinstance(parsed, dict):
                return parsed
            return {}
        except json.JSONDecodeError:
            return {}

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
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.robot_yaw = float(yaw)
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        self.robot_speed = math.hypot(vx, vy)

    def _automated_cb(self, msg: Bool) -> None:
        self.automated_enabled = bool(msg.data)

    def _deadman_cb(self, msg: Bool) -> None:
        self.deadman_pressed = bool(msg.data)

    def _estop_cb(self, msg: Bool) -> None:
        self.estop = bool(msg.data)

    def _photo_done_cb(self, msg: String) -> None:
        payload = self._parse_photo_done(msg.data)
        current_wp = self._current_waypoint()
        if current_wp is None:
            return
        waypoint_id = str(payload.get("waypoint_id", ""))
        if waypoint_id and waypoint_id != current_wp.waypoint_id:
            return
        self.photo_done_for_waypoint = bool(payload.get("success", True))
        self._publish_event(
            "photo_done",
            {"success": self.photo_done_for_waypoint, "photo_path": payload.get("photo_path")},
            current_wp.waypoint_id,
        )

    def _current_waypoint(self):
        if self.plan is None:
            return None
        if self.current_idx < 0 or self.current_idx >= len(self.plan.waypoints):
            return None
        return self.plan.waypoints[self.current_idx]

    def _is_safety_ready(self) -> bool:
        if self.estop:
            return False
        if self.require_safety_topics:
            return self.automated_enabled and self.deadman_pressed
        if self.autostart:
            return True
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
        try:
            self.plan = self.loader.build_plan(datum=datum, fallback_home=fallback_home)
        except RuntimeError as exc:
            self.get_logger().error(f"Failed to build mission plan: {exc}")
            self._publish_event("plan_failed", {"error": str(exc)})
            self._set_state("ABORTED")
            return False
        self._publish_event(
            "datum_set",
            {
                "lat": self.plan.datum.lat,
                "lon": self.plan.datum.lon,
                "alt": self.plan.datum.alt,
                "source": "yaml" if self.loader.yaml_datum is not None else "first_fix",
            },
        )
        self._publish_event("plan_loaded", {"mission_id": self.plan.mission_id, "count": len(self.plan.waypoints)})
        return True

    def _publish_current_waypoint(self) -> None:
        wp = self._current_waypoint()
        if wp is None:
            return
        msg = String()
        msg.data = json.dumps(
            {
                "id": wp.waypoint_id,
                "lat": wp.lat,
                "lon": wp.lon,
                "x": wp.x,
                "y": wp.y,
                "arrival_radius": wp.arrival_radius,
                "pass_side": wp.pass_side,
                "policy": wp.policy,
                "photo_required": wp.photo_required,
            },
            ensure_ascii=False,
        )
        self.current_wp_pub.publish(msg)

    def _build_goal_pose(self):
        wp = self._current_waypoint()
        if wp is None or self.plan is None:
            return None
        policy = self.plan.policies[wp.policy]
        lateral = policy.goal_offset_lateral
        longitudinal = policy.goal_offset_longitudinal
        if abs(lateral) < 1e-6 and wp.pass_side != "none":
            lateral = wp.arrival_radius
        if abs(longitudinal) < 1e-6:
            longitudinal = wp.arrival_radius
        offset = GoalOffset(lateral=lateral, longitudinal=longitudinal)

        if wp.yaw_hint is not None:
            theta_in = wp.yaw_hint
        elif self.current_idx > 0:
            prev = self.plan.waypoints[self.current_idx - 1]
            theta_in = math.atan2(wp.y - prev.y, wp.x - prev.x)
        else:
            theta_in = math.atan2(wp.y - self.robot_y, wp.x - self.robot_x)
            if math.isnan(theta_in):
                theta_in = 0.0

        gx, gy, gyaw = compute_goal_xyyaw(
            cone_x=wp.x,
            cone_y=wp.y,
            theta_in=theta_in,
            pass_side=wp.pass_side,
            offset=offset,
        )
        pose = build_pose_stamped(
            frame_id=self.frame_id,
            stamp=self.get_clock().now().to_msg(),
            x=gx,
            y=gy,
            yaw=gyaw,
        )
        return pose, offset, theta_in

    def _send_goal(self) -> None:
        if self.current_goal_sent:
            return
        wp = self._current_waypoint()
        if wp is None:
            self._set_state("COMPLETED")
            return
        if not self.action_client.wait_for_server(timeout_sec=0.1):
            return

        built = self._build_goal_pose()
        if built is None:
            self._set_state("ABORTED")
            return
        goal_pose, offset, theta_in = built
        self._publish_current_waypoint()

        self.arrival_judge = ArrivalJudge(confirm_frames=self.plan.policies[wp.policy].confirm_frames)
        self.nav_goal_reached = False
        self.nav_failed = False
        self.photo_done_for_waypoint = not wp.photo_required
        self.photo_deadline_sec = self._now_sec() + self.photo_timeout_sec

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        self.current_goal_sent = True
        self.current_goal_send_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_cb)
        self.current_goal_send_future.add_done_callback(self._goal_response_cb)
        self._publish_event(
            "goal_sent",
            {
                "goal_x": goal_pose.pose.position.x,
                "goal_y": goal_pose.pose.position.y,
                "goal_yaw": theta_in,
                "offset": asdict(offset),
            },
            waypoint_id=wp.waypoint_id,
        )

    def _feedback_cb(self, _feedback_msg) -> None:
        # Feedback is currently not used for transitions, but kept for observability.
        return

    def _goal_response_cb(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.current_goal_sent = False
            self.nav_failed = True
            self.get_logger().error(f"NavigateToPose send failed: {exc}")
            return

        if not goal_handle.accepted:
            self.current_goal_sent = False
            self.nav_failed = True
            wp = self._current_waypoint()
            self._publish_event("goal_rejected", {}, wp.waypoint_id if wp else None)
            return

        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)
        self._set_state("NAVIGATING")

    def _goal_result_cb(self, future) -> None:
        self.current_goal_sent = False
        try:
            wrapped = future.result()
        except Exception as exc:  # noqa: BLE001
            self.nav_failed = True
            self.get_logger().error(f"NavigateToPose result failed: {exc}")
            return

        status = int(wrapped.status)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.nav_goal_reached = True
            wp = self._current_waypoint()
            self._publish_event("nav2_goal_succeeded", {}, wp.waypoint_id if wp else None)
            return
        self.nav_failed = True
        wp = self._current_waypoint()
        self._publish_event("nav2_goal_failed", {"status": status}, wp.waypoint_id if wp else None)

    def _cancel_goal(self) -> None:
        if self.current_goal_handle is None:
            return
        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(lambda _f: None)
        self.current_goal_handle = None
        self.current_goal_sent = False

    def _distance_to_current_waypoint(self) -> Optional[float]:
        wp = self._current_waypoint()
        if wp is None:
            return None
        return math.hypot(self.robot_x - wp.x, self.robot_y - wp.y)

    def _loop(self) -> None:
        if self.state in {"COMPLETED", "ABORTED"}:
            if not self.completed_announced:
                self.completed_announced = True
                self._publish_event("mission_finished", {"outcome": self.state.lower()})
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
            if self.gps_health != "HEALTHY":
                return
            if not self.global_odom_ready:
                return
            self._set_state("WAITING_FOR_SAFETY")
            return

        if self.state == "WAITING_FOR_SAFETY":
            if self.gps_health == "FATAL":
                self._set_state("SAFE_STOP")
                return
            if self._is_safety_ready():
                self._set_state("PLANNING_NEXT")
            return

        if self.state == "SAFE_STOP":
            if self.gps_health == "FATAL":
                return
            if self._is_safety_ready():
                fallback = self.prev_state if self.prev_state not in {None, "SAFE_STOP"} else "PLANNING_NEXT"
                if fallback in {"NAVIGATING", "ARRIVING", "LEAVING"}:
                    fallback = "PLANNING_NEXT"
                self._set_state(fallback)
            return

        if self.state == "PLANNING_NEXT":
            if self.plan is None:
                self._set_state("ABORTED")
                return
            if self.current_idx >= len(self.plan.waypoints):
                self._set_state("COMPLETED")
                return
            if self.nav_failed:
                wp = self._current_waypoint()
                if wp is None:
                    self._set_state("ABORTED")
                    return
                if self.nav_retry_count < 1 and not wp.is_home_return:
                    self.nav_retry_count += 1
                    self.nav_failed = False
                    self._publish_event("goal_retry", {"retry": self.nav_retry_count}, wp.waypoint_id)
                elif wp.is_home_return:
                    self._publish_event("home_return_failed", {}, wp.waypoint_id)
                    self._set_state("ABORTED")
                    return
                else:
                    self._publish_event("waypoint_failed_skip", {}, wp.waypoint_id)
                    self.current_idx += 1
                    self.nav_retry_count = 0
                    self.nav_failed = False
                    return
            self._send_goal()
            return

        if self.state == "NAVIGATING":
            wp = self._current_waypoint()
            if wp is None:
                self._set_state("ABORTED")
                return
            if self.gps_health == "FATAL":
                self._cancel_goal()
                self._set_state("SAFE_STOP")
                return
            if self.nav_failed:
                if self.nav_retry_count < 1 and not wp.is_home_return:
                    self.nav_retry_count += 1
                    self._publish_event("nav_retry", {"retry": self.nav_retry_count}, wp.waypoint_id)
                    self._set_state("PLANNING_NEXT")
                elif wp.is_home_return:
                    self._publish_event("home_return_failed", {}, wp.waypoint_id)
                    self._set_state("ABORTED")
                else:
                    self._publish_event("waypoint_failed_skip", {}, wp.waypoint_id)
                    self.current_idx += 1
                    self.nav_retry_count = 0
                    self._set_state("PLANNING_NEXT")
                self.nav_failed = False
                self.current_goal_sent = False
                self.current_goal_handle = None
                return

            now_sec = self._now_sec()
            arrived, dist_m = self.arrival_judge.update(
                now_sec=now_sec,
                robot_x=self.robot_x,
                robot_y=self.robot_y,
                robot_yaw=self.robot_yaw,
                linear_speed_mps=self.robot_speed,
                target_x=wp.x,
                target_y=wp.y,
                arrival_radius=wp.arrival_radius,
                gps_horizontal_std=self.gps_std_m,
                nav2_goal_reached=self.nav_goal_reached,
                vision_override=False,
            )
            if arrived:
                self._publish_event("arrived", {"distance_m": round(dist_m, 3)}, wp.waypoint_id)
                self._cancel_goal()
                if wp.photo_required:
                    request = String()
                    request.data = json.dumps(
                        {
                            "waypoint_id": wp.waypoint_id,
                            "pose": {
                                "x": self.robot_x,
                                "y": self.robot_y,
                                "yaw": self.robot_yaw,
                            },
                        },
                        ensure_ascii=False,
                    )
                    self.photo_request_pub.publish(request)
                    self._publish_event("photo_request", {}, wp.waypoint_id)
                self._set_state("ARRIVING")
            return

        if self.state == "ARRIVING":
            wp = self._current_waypoint()
            if wp is None:
                self._set_state("ABORTED")
                return
            now_sec = self._now_sec()
            if not self.photo_done_for_waypoint and now_sec > self.photo_deadline_sec:
                self.photo_done_for_waypoint = True
                self._publish_event("photo_timeout", {"timeout_sec": self.photo_timeout_sec}, wp.waypoint_id)

            if self.photo_done_for_waypoint:
                side_ok = self.arrival_judge.verify_pass_side(wp.x, wp.y, wp.pass_side)
                self._publish_event("pass_side_verified", {"ok": side_ok}, wp.waypoint_id)
                if not side_ok:
                    self._publish_event("pass_side_violated", {}, wp.waypoint_id)
                self._set_state("LEAVING")
            return

        if self.state == "LEAVING":
            wp = self._current_waypoint()
            if wp is None:
                self._set_state("ABORTED")
                return
            dist = self._distance_to_current_waypoint()
            if dist is None:
                return
            if dist > (wp.arrival_radius + self.leave_buffer_m):
                self.current_idx += 1
                self.nav_retry_count = 0
                self.nav_goal_reached = False
                self.photo_done_for_waypoint = False
                self.arrival_judge.reset()
                self._set_state("PLANNING_NEXT")


def main() -> None:
    rclpy.init()
    node = MissionOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
