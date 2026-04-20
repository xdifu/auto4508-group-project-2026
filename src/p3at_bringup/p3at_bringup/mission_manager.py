#!/usr/bin/env python3
"""Send a FollowWaypoints goal and publish the mission waypoints for RViz."""
import subprocess
from pathlib import Path
from typing import List

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path as PathMsg
from rclpy.action import ActionClient
from rclpy.node import Node
from tf_transformations import quaternion_from_euler


class MissionManager(Node):
    def __init__(self) -> None:
        super().__init__("mission_manager")
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("autostart", True)
        self.declare_parameter("start_delay_sec", 5.0)

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.waypoints_file = self.get_parameter("waypoints_file").get_parameter_value().string_value
        self.autostart = self.get_parameter("autostart").get_parameter_value().bool_value
        self.start_delay_sec = self.get_parameter("start_delay_sec").get_parameter_value().double_value

        self.action_client = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self.path_pub = self.create_publisher(PathMsg, "/mission_waypoints", 10)
        self.goal_sent = False
        self.mission_finished = False
        self.map_received = False
        self.ready_logged = False
        self.last_feedback_index = None
        self.start_time_ns = None
        self.waypoints = self._load_waypoints()
        self.path_msg = self._build_path_message(self.waypoints)
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, 10)
        self.publish_timer = self.create_timer(1.0, self._publish_path)
        self.start_timer = self.create_timer(1.0, self._maybe_start)

    def _load_waypoints(self) -> List[PoseStamped]:
        if not self.waypoints_file:
            raise RuntimeError("Parameter 'waypoints_file' is required.")

        waypoint_path = Path(self.waypoints_file)
        data = yaml.safe_load(waypoint_path.read_text(encoding="utf-8"))
        frame_id = data.get("waypoints", {}).get("frame_id", self.frame_id)
        waypoints = []
        for item in data.get("waypoints", {}).get("poses", []):
            yaw = float(item["yaw"])
            quat = quaternion_from_euler(0.0, 0.0, yaw)

            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = float(item["x"])
            pose.pose.position.y = float(item["y"])
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            waypoints.append(pose)

        if not waypoints:
            raise RuntimeError(f"No waypoints found in {waypoint_path}")
        return waypoints

    def _build_path_message(self, waypoints: List[PoseStamped]) -> PathMsg:
        path = PathMsg()
        path.header.frame_id = waypoints[0].header.frame_id
        path.poses = waypoints
        return path

    def _publish_path(self) -> None:
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        for pose in self.path_msg.poses:
            pose.header.stamp = self.path_msg.header.stamp
        self.path_pub.publish(self.path_msg)

    def _map_cb(self, _msg: OccupancyGrid) -> None:
        self.map_received = True
        if self.start_time_ns is None:
            self.start_time_ns = self.get_clock().now().nanoseconds

    def _maybe_start(self) -> None:
        if not self.autostart or self.goal_sent or self.mission_finished:
            return
        if not self.map_received:
            if not self.ready_logged:
                self.get_logger().info("Waiting for first /map message before starting mission.")
                self.ready_logged = True
            return
        if self.start_time_ns is None:
            return
        elapsed = (self.get_clock().now().nanoseconds - self.start_time_ns) / 1e9
        if elapsed < self.start_delay_sec:
            return
        if not self.action_client.wait_for_server(timeout_sec=0.1):
            if not self.ready_logged:
                self.get_logger().info("Waiting for FollowWaypoints action server...")
                self.ready_logged = True
            return
        self.ready_logged = False

        goal = FollowWaypoints.Goal()
        goal.poses = self.waypoints
        self.goal_sent = True
        send_future = self.action_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)
        self.get_logger().info(f"Sent mission with {len(self.waypoints)} waypoints.")

    def _feedback_cb(self, feedback_msg) -> None:
        current_index = feedback_msg.feedback.current_waypoint
        if current_index != self.last_feedback_index:
            self.last_feedback_index = current_index
            self.get_logger().info(f"Current waypoint index: {current_index}")

    def _goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.goal_sent = False
            self.get_logger().warning("FollowWaypoints goal rejected. Retrying...")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        result = future.result().result
        self.goal_sent = False
        self.mission_finished = True
        self.last_feedback_index = None
        if result.missed_waypoints:
            self.get_logger().warning(f"Missed waypoint indices: {result.missed_waypoints}")
        else:
            self.get_logger().info("Mission completed successfully.")
        self._save_map()

    def _save_map(self) -> None:
        """Save the SLAM map to disk after mission completion."""
        map_file = str(Path.cwd() / "part1_slam_map")
        self.get_logger().info(f"Saving SLAM map to {map_file} ...")
        try:
            proc = subprocess.run(
                [
                    "ros2", "run", "nav2_map_server", "map_saver_cli",
                    "-f", map_file,
                    "--ros-args",
                    "-p", "use_sim_time:=true",
                    "-p", "save_map_timeout:=10.0",
                    "-p", "free_thresh_default:=0.25",
                    "-p", "occupied_thresh_default:=0.65",
                ],
                capture_output=True,
                text=True,
                timeout=30,
            )
            if proc.returncode == 0:
                self.get_logger().info(f"Map saved: {map_file}.pgm / {map_file}.yaml")
            else:
                self.get_logger().warning(
                    f"map_saver_cli returned {proc.returncode}: {proc.stderr.strip()}"
                )
        except subprocess.TimeoutExpired:
            self.get_logger().warning("Map save timed out.")
        except FileNotFoundError:
            self.get_logger().warning("map_saver_cli not found; install nav2_map_server.")
        except Exception as exc:
            self.get_logger().warning(f"Map save error: {exc}")


def main() -> None:
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
