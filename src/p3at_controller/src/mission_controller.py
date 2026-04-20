#!/usr/bin/env python3
import math
import os
import json
import time
import yaml
from datetime import datetime

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        # ----------------------------
        # Default navigation parameters
        # ----------------------------
        self.max_linear = 0.18
        self.max_angular = 0.80
        self.obstacle_stop_dist = 0.50
        self.max_wp_nav_time = 60.0
        self.vision_wait_timeout = 3.0

        # keep-object-on-right placeholder guidance
        self.desired_right_dist = 0.80
        self.right_bias_gain = 0.90
        self.right_bias_max = 0.25

        # ----------------------------
        # Load config
        # ----------------------------
        cfg_path = os.path.join(
            os.path.dirname(__file__),
            '..',
            'config',
            'waypoints_part2.yaml'
        )

        with open(cfg_path, 'r') as f:
            data = yaml.safe_load(f)

        self.mission_id = data.get('mission_id', 'part2_run')
        self.pose_source_topic = data.get('pose_source', {}).get('topic', '/odom')

        self.waypoints = data['waypoints']
        self.home = data['home']

        weave_cfg = data.get('weave', {})
        self.weave_enabled = bool(weave_cfg.get('enabled', True))
        self.weave_amplitude = float(weave_cfg.get('amplitude', 0.35))
        self.weave_num_offsets = int(weave_cfg.get('num_offsets', 4))

        nav_cfg = data.get('navigation', {})
        self.max_wp_nav_time = float(nav_cfg.get('max_wp_nav_time', self.max_wp_nav_time))
        self.vision_wait_timeout = float(nav_cfg.get('vision_wait_timeout', self.vision_wait_timeout))
        self.max_linear = float(nav_cfg.get('max_linear', self.max_linear))
        self.max_angular = float(nav_cfg.get('max_angular', self.max_angular))
        self.obstacle_stop_dist = float(nav_cfg.get('obstacle_stop_dist', self.obstacle_stop_dist))

        dev_cfg = data.get('development', {})
        self.auto_start = bool(dev_cfg.get('auto_start', True))

        topics_cfg = data.get('topics', {})
        self.cmd_vel_auto_topic = topics_cfg.get('cmd_vel_auto', '/cmd_vel_auto')
        self.nav_log_topic = topics_cfg.get('nav_log', '/mission/navigation')
        self.vision_log_topic = topics_cfg.get('vision_log', '/mission/vision')

        output_cfg = data.get('output', {})
        self.workspace_root = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', '..', '..')
        )
        base_dir = output_cfg.get(
            'base_dir',
            os.path.join(self.workspace_root, 'part2_runs')
        )
        run_stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.run_dir = os.path.join(base_dir, f'{self.mission_id}_{run_stamp}')
        self.logs_dir = os.path.join(self.run_dir, 'logs')
        os.makedirs(self.logs_dir, exist_ok=True)

        self.summary_file_path = None

        # ----------------------------
        # Topics
        # ----------------------------
        self.cmd_auto_pub = self.create_publisher(Twist, self.cmd_vel_auto_topic, 10)
        self.nav_log_pub = self.create_publisher(String, self.nav_log_topic, 10)

        self.odom_sub = self.create_subscription(
            Odometry, self.pose_source_topic, self.odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.vision_sub = self.create_subscription(
            String, self.vision_log_topic, self.vision_callback, 10
        )

        # ----------------------------
        # Robot state
        # ----------------------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_ready = False
        self.scan_ready = False

        self.front_min = float('inf')
        self.left_min = float('inf')
        self.right_min = float('inf')

        # ----------------------------
        # Mission state
        # ----------------------------
        self.state = 'IDLE'
        self.current_wp_idx = 0
        self.current_wp_start_time = time.time()
        self.pending_vision_wp_id = None
        self.pending_vision_deadline = None

        self.weave_points = []
        self.weave_idx = 0

        self.mission_started = False
        self.mission_status = 'running'
        self.mission_start_time = time.time()
        self.summary_printed = False

        self.vision_results = {}
        self.mission_log = []
        self.generated_images = []

        self.get_logger().info(f'Mission ID: {self.mission_id}')
        self.get_logger().info(f'Loaded waypoints: {self.waypoints}')
        self.get_logger().info(f'Home: {self.home}')
        self.get_logger().info(f'Pose source: {self.pose_source_topic}')
        self.get_logger().info(f'Weave enabled: {self.weave_enabled}')
        self.get_logger().info(f'Auto start: {self.auto_start}')
        self.get_logger().info(f'Run dir: {self.run_dir}')

        self.timer = self.create_timer(0.1, self.control_loop)

    # -------------------------------------------------
    # ROS callbacks
    # -------------------------------------------------
    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        self.odom_ready = True

    def scan_callback(self, msg: LaserScan):
        self.front_min = self.scan_min_in_sector(msg, -20.0, 20.0)
        self.left_min = self.scan_min_in_sector(msg, 60.0, 100.0)
        self.right_min = self.scan_min_in_sector(msg, -100.0, -60.0)
        self.scan_ready = True

    def vision_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Invalid vision JSON ignored: {e}')
            return

        wp_id = data.get('wp_id')
        if wp_id is None:
            self.get_logger().warn('Vision result without wp_id ignored.')
            return

        self.vision_results[int(wp_id)] = data
        self.get_logger().info(f'Cached vision result for waypoint {wp_id}: {data}')

    # -------------------------------------------------
    # JSON publish helpers
    # -------------------------------------------------
    def _publish_json_string(self, publisher, data_dict):
        msg = String()
        msg.data = json.dumps(data_dict, ensure_ascii=False)
        publisher.publish(msg)

    def publish_navigation_log(self, wp_id, status, extra=None):
        data_dict = {
            "wp_id": int(wp_id),
            "status": str(status),
            "time": time.time()
        }
        if extra:
            data_dict.update(extra)
        self._publish_json_string(self.nav_log_pub, data_dict)

    # -------------------------------------------------
    # Scan helpers
    # -------------------------------------------------
    def scan_min_in_sector(self, msg: LaserScan, angle_deg_min, angle_deg_max):
        a0 = math.radians(angle_deg_min)
        a1 = math.radians(angle_deg_max)

        values = []
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if a0 <= angle <= a1:
                values.append(r)

        if not values:
            return float('inf')
        return min(values)

    def compute_right_side_bias(self):
        if math.isinf(self.right_min) or math.isnan(self.right_min):
            return 0.0

        if self.right_min > 2.0:
            return 0.0

        err = self.desired_right_dist - self.right_min
        bias = self.right_bias_gain * err
        return clamp(bias, -self.right_bias_max, self.right_bias_max)

    # -------------------------------------------------
    # Motion helpers
    # -------------------------------------------------
    def publish_stop(self):
        self.cmd_auto_pub.publish(Twist())

    def go_to_target(self, target_x, target_y, arrival_radius, apply_right_bias=False):
        dx = target_x - self.x
        dy = target_y - self.y
        dist = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        heading_err = normalize_angle(target_yaw - self.yaw)

        if dist < arrival_radius:
            return True

        if self.front_min < self.obstacle_stop_dist:
            self.get_logger().warn(
                f'Obstacle too close in front: {self.front_min:.2f} m, stopping.'
            )
            self.publish_stop()
            return False

        right_bias = self.compute_right_side_bias() if apply_right_bias else 0.0

        cmd = Twist()
        cmd.angular.z = clamp(
            1.6 * heading_err + right_bias,
            -self.max_angular,
            self.max_angular
        )

        if abs(heading_err) > 0.60:
            cmd.linear.x = 0.05
        else:
            cmd.linear.x = min(self.max_linear, 0.10 + 0.15 * dist)

        self.cmd_auto_pub.publish(cmd)
        return False

    # -------------------------------------------------
    # Mission helpers
    # -------------------------------------------------
    def start_mission(self):
        if self.mission_started:
            return

        if not self.waypoints:
            self.get_logger().error('No waypoints loaded.')
            self.mission_status = 'failed'
            self.state = 'DONE'
            return

        self.mission_started = True
        self.current_wp_idx = 0
        self.state = 'NAVIGATE'
        self.current_wp_start_time = time.time()

        wp = self.waypoints[self.current_wp_idx]
        self.publish_navigation_log(
            self.current_wp_idx + 1,
            'searching',
            {
                'target_x': float(wp['x']),
                'target_y': float(wp['y']),
                'arrival_radius': float(wp.get('arrival_radius', 0.8)),
                'pass_side': wp.get('pass_side', 'none')
            }
        )
        self.get_logger().info('Mission started.')

    def build_unknown_vision_result(self, wp_id, status='timeout'):
        return {
            'wp_id': int(wp_id),
            'status': status,
            'obj_color': 'unknown',
            'obj_shape': 'unknown',
            'dist': -1.0,
            'img_path': ''
        }

    def finalize_current_waypoint(self, vision_result):
        wp = self.waypoints[self.current_wp_idx]
        img_path = vision_result.get('img_path', '')
        if img_path:
            self.generated_images.append(img_path)

        self.mission_log.append({
            'waypoint_index': self.current_wp_idx + 1,
            'waypoint_id': wp.get('id', f'WP{self.current_wp_idx + 1}'),
            'target_x': wp['x'],
            'target_y': wp['y'],
            'arrival_radius': float(wp.get('arrival_radius', 0.8)),
            'pass_side_expected': wp.get('pass_side', 'none'),
            'policy': wp.get('policy', 'default'),
            'vision_result': vision_result,
        })

        self.pending_vision_wp_id = None
        self.pending_vision_deadline = None

        if self.current_wp_idx == 0 and self.weave_enabled:
            self.current_wp_idx += 1
            entered = self.maybe_enter_weave()
            if entered:
                return

        if self.current_wp_idx < len(self.waypoints) - 1:
            self.current_wp_idx += 1
            self.state = 'NAVIGATE'
            self.current_wp_start_time = time.time()

            next_wp = self.waypoints[self.current_wp_idx]
            self.publish_navigation_log(
                self.current_wp_idx + 1,
                'searching',
                {
                    'target_x': float(next_wp['x']),
                    'target_y': float(next_wp['y']),
                    'arrival_radius': float(next_wp.get('arrival_radius', 0.8)),
                    'pass_side': next_wp.get('pass_side', 'none')
                }
            )
            self.get_logger().info(
                f"Moving to next waypoint {next_wp.get('id', self.current_wp_idx + 1)}: {next_wp}"
            )
        else:
            self.state = 'RETURN_HOME'
            self.get_logger().info('All mission waypoints completed. Returning home.')

    # -------------------------------------------------
    # Weave
    # -------------------------------------------------
    def generate_weave_points(self, start_pt, end_pt, amplitude=0.35, num_offsets=4):
        x0, y0 = start_pt
        x1, y1 = end_pt

        dx = x1 - x0
        dy = y1 - y0
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return []

        ux = dx / length
        uy = dy / length
        px = -uy
        py = ux

        points = []
        total_segments = num_offsets + 1
        for i in range(1, num_offsets + 1):
            t = i / total_segments
            base_x = x0 + t * dx
            base_y = y0 + t * dy
            sign = 1.0 if i % 2 == 1 else -1.0
            wx = base_x + sign * amplitude * px
            wy = base_y + sign * amplitude * py
            points.append((wx, wy))

        return points

    def maybe_enter_weave(self):
        if not self.weave_enabled:
            return False

        if self.current_wp_idx != 1:
            return False

        start_wp = self.waypoints[0]
        end_wp = self.waypoints[1]

        start_pt = (start_wp['x'], start_wp['y'])
        end_pt = (end_wp['x'], end_wp['y'])

        self.weave_points = self.generate_weave_points(
            start_pt,
            end_pt,
            amplitude=self.weave_amplitude,
            num_offsets=self.weave_num_offsets
        )

        if not self.weave_points:
            return False

        self.weave_idx = 0
        self.state = 'WEAVE'
        self.get_logger().info(f'Entering WEAVE state with points: {self.weave_points}')
        return True

    # -------------------------------------------------
    # Summary
    # -------------------------------------------------
    def save_summary_to_file(self):
        mission_end_time = time.time()

        payload = {
            'summary_version': 'integrated_v1',
            'mission_status': self.mission_status,
            'mission_start_unix': self.mission_start_time,
            'mission_end_unix': mission_end_time,
            'mission_start_iso': datetime.fromtimestamp(self.mission_start_time).isoformat(),
            'mission_end_iso': datetime.fromtimestamp(mission_end_time).isoformat(),
            'duration_sec': round(mission_end_time - self.mission_start_time, 3),
            'returned_home': self.mission_status == 'completed',
            'weave_enabled': self.weave_enabled,
            'generated_images': self.generated_images,
            'waypoint_results': self.mission_log,
        }

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        out_path = os.path.join(self.logs_dir, f'journey_summary_{timestamp}.json')

        with open(out_path, 'w') as f:
            json.dump(payload, f, indent=2)

        self.summary_file_path = out_path
        self.get_logger().info(f'Saved journey summary to: {out_path}')
        return out_path

    def print_summary_once(self):
        if self.summary_printed:
            return

        self.summary_printed = True
        self.get_logger().info('================ JOURNEY SUMMARY ================')
        for item in self.mission_log:
            self.get_logger().info(str(item))
        saved_path = self.save_summary_to_file()
        self.get_logger().info(f'Summary file: {saved_path}')
        self.get_logger().info(f'Mission ended with status: {self.mission_status}')
        self.get_logger().info('================================================')

    # -------------------------------------------------
    # Main control loop
    # -------------------------------------------------
    def control_loop(self):
        if not self.odom_ready or not self.scan_ready:
            self.publish_stop()
            return

        if not self.mission_started:
            if self.auto_start:
                self.start_mission()
            else:
                self.publish_stop()
                return

        if self.state == 'IDLE':
            self.publish_stop()
            return

        if self.state == 'NAVIGATE':
            wp = self.waypoints[self.current_wp_idx]
            target_x = wp['x']
            target_y = wp['y']
            arrival_radius = float(wp.get('arrival_radius', 0.8))
            pass_side = wp.get('pass_side', 'none')

            if time.time() - self.current_wp_start_time > self.max_wp_nav_time:
                self.publish_navigation_log(
                    self.current_wp_idx + 1,
                    'failed',
                    {
                        'target_x': float(target_x),
                        'target_y': float(target_y),
                        'reason': 'timeout'
                    }
                )
                self.mission_status = 'failed'
                self.publish_stop()
                self.state = 'DONE'
                self.get_logger().warn(
                    f"Waypoint {wp.get('id', self.current_wp_idx + 1)} navigation timeout."
                )
                return

            reached = self.go_to_target(
                target_x,
                target_y,
                arrival_radius,
                apply_right_bias=(pass_side == 'right')
            )

            if reached:
                self.publish_stop()
                self.publish_navigation_log(
                    self.current_wp_idx + 1,
                    'reached',
                    {
                        'target_x': float(target_x),
                        'target_y': float(target_y),
                        'arrival_radius': float(arrival_radius),
                        'pass_side': pass_side
                    }
                )
                self.pending_vision_wp_id = self.current_wp_idx + 1
                self.pending_vision_deadline = time.time() + self.vision_wait_timeout
                self.state = 'WAIT_VISION'
                self.get_logger().info(
                    f"Reached waypoint {wp.get('id', self.current_wp_idx + 1)}: ({target_x}, {target_y})"
                )
                return

        elif self.state == 'WAIT_VISION':
            self.publish_stop()

            if self.pending_vision_wp_id in self.vision_results:
                vision_result = self.vision_results.pop(self.pending_vision_wp_id)
                self.finalize_current_waypoint(vision_result)
                return

            if time.time() > self.pending_vision_deadline:
                self.get_logger().warn(
                    f'Vision timeout for waypoint {self.pending_vision_wp_id}, continuing with unknown result.'
                )
                vision_result = self.build_unknown_vision_result(
                    self.pending_vision_wp_id,
                    status='timeout'
                )
                self.finalize_current_waypoint(vision_result)
                return

            return

        elif self.state == 'WEAVE':
            if self.weave_idx < len(self.weave_points):
                target = self.weave_points[self.weave_idx]
                reached = self.go_to_target(
                    target[0],
                    target[1],
                    0.30,
                    apply_right_bias=True
                )

                if reached:
                    self.publish_stop()
                    self.get_logger().info(
                        f'Reached weave point {self.weave_idx + 1}: {target}'
                    )
                    self.weave_idx += 1
            else:
                self.state = 'NAVIGATE'
                self.current_wp_start_time = time.time()
                wp = self.waypoints[self.current_wp_idx]
                self.publish_navigation_log(
                    self.current_wp_idx + 1,
                    'searching',
                    {
                        'target_x': float(wp['x']),
                        'target_y': float(wp['y']),
                        'arrival_radius': float(wp.get('arrival_radius', 0.8)),
                        'pass_side': wp.get('pass_side', 'none')
                    }
                )
                self.get_logger().info(
                    f'WEAVE complete. Continuing to waypoint {self.current_wp_idx + 1}: {wp}'
                )

        elif self.state == 'RETURN_HOME':
            reached_home = self.go_to_target(
                self.home['x'],
                self.home['y'],
                float(self.home.get('arrival_radius', 0.8)),
                apply_right_bias=False
            )

            if reached_home:
                self.publish_stop()
                self.publish_navigation_log(
                    0,
                    'home_reached',
                    {
                        'home_x': float(self.home['x']),
                        'home_y': float(self.home['y'])
                    }
                )
                self.mission_status = 'completed'
                self.state = 'DONE'
                self.get_logger().info('Reached home position.')

        elif self.state == 'DONE':
            self.publish_stop()
            self.print_summary_once()


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.mission_status = 'aborted'
        node.state = 'DONE'
    finally:
        try:
            node.publish_stop()
        except Exception:
            pass

        try:
            node.print_summary_once()
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
