#!/usr/bin/env python3
import math
import os
import json
import time
from turtle import shape
from unittest import result
import yaml
from datetime import datetime

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

try:
    from sensor_msgs.msg import Joy
    JOY_AVAILABLE = True
except Exception:
    JOY_AVAILABLE = False


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
        # Parameters
        # ----------------------------
        self.wp_tolerance = 0.30
        self.max_linear = 0.18
        self.max_angular = 0.80
        self.obstacle_stop_dist = 0.50
        
        # simplified "keep marker on right" parameters
        self.desired_right_dist = 0.80
        self.right_bias_gain = 0.90
        self.right_bias_max = 0.25

        # manual control placeholders
        self.manual_linear_scale = 0.20
        self.manual_angular_scale = 0.80

        # joystick mapping placeholders
        self.BTN_X = 0
        self.BTN_O = 1
        self.BTN_DEADMAN = 5
        self.AXIS_LINEAR = 1
        self.AXIS_ANGULAR = 3

        # ----------------------------
        # Topics
        # ----------------------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        if JOY_AVAILABLE:
            self.joy_sub = self.create_subscription(
                Joy, '/joy', self.joy_callback, 10
            )
        else:
            self.joy_sub = None

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
        # Joy state
        # ----------------------------
        self.joy_ready = False
        self.axes = []
        self.buttons = []
        self.deadman_pressed = False
        self.last_x_pressed = False
        self.last_o_pressed = False

        # ----------------------------
        # Mission state
        # ----------------------------
        self.mode = 'AUTO'
        self.state = 'NAVIGATE'
        self.current_wp_idx = 0
        self.summary_printed = False
        self.mission_log = []
        self.last_marker_photo = None
        self.mission_start_time = time.time()
        self.mission_status = 'running'
        self.generated_images = []
        self.summary_file_path = None

        # weave state
        self.weave_points = []
        self.weave_idx = 0

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

        self.waypoints = [tuple(wp) for wp in data['waypoints']]
        self.home = tuple(data['home'])

        weave_cfg = data.get('weave', {})
        self.weave_enabled = weave_cfg.get('enabled', True)
        self.weave_only_between_1_and_2 = weave_cfg.get('only_between_waypoint_1_and_2', True)
        self.weave_amplitude = float(weave_cfg.get('amplitude', 0.35))
        self.weave_num_offsets = int(weave_cfg.get('num_offsets', 4))

        dev_cfg = data.get('development', {})
        self.allow_auto_without_joy = bool(dev_cfg.get('allow_auto_without_joy', True))

        self.logs_dir = os.path.join(
            os.path.dirname(__file__),
            '..',
            'logs'
        )
        os.makedirs(self.logs_dir, exist_ok=True)
        
        self.images_dir = os.path.join(
            os.path.dirname(__file__),
            '..',
            'images_out'
        )
        os.makedirs(self.images_dir, exist_ok=True)

        self.workspace_root = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', '..', '..')
        )
        self.path_plot_path = os.path.join(self.workspace_root, 'driven_path_plot.png')

        self.get_logger().info(f'Loaded waypoints: {self.waypoints}')
        self.get_logger().info(f'Home: {self.home}')
        self.get_logger().info(f'Weave enabled: {self.weave_enabled}')
        self.get_logger().info(f'allow_auto_without_joy: {self.allow_auto_without_joy}')
        self.get_logger().info(f'Start mode: {self.mode}')

        self.timer = self.create_timer(0.1, self.control_loop)

    # -------------------------------------------------
    # Callbacks
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

    def joy_callback(self, msg):
        self.axes = list(msg.axes)
        self.buttons = list(msg.buttons)
        self.joy_ready = True

        x_now = self.button_pressed(self.BTN_X)
        o_now = self.button_pressed(self.BTN_O)

        if x_now and not self.last_x_pressed:
            self.mode = 'AUTO'
            self.get_logger().info('Switched to AUTO mode.')

        if o_now and not self.last_o_pressed:
            self.mode = 'MANUAL'
            self.get_logger().info('Switched to MANUAL mode.')

        self.last_x_pressed = x_now
        self.last_o_pressed = o_now
        self.deadman_pressed = self.button_pressed(self.BTN_DEADMAN)

    # -------------------------------------------------
    # Joy helpers
    # -------------------------------------------------
    def button_pressed(self, idx):
        return idx < len(self.buttons) and self.buttons[idx] == 1

    def axis_value(self, idx):
        if idx < len(self.axes):
            return self.axes[idx]
        return 0.0

    # -------------------------------------------------
    # Scan utility
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
        """
        Simplified right-side guidance using lidar.
        If the object/wall on the right is too close, turn slightly left.
        If it is too far, turn slightly right.
        This is a placeholder for the Part 2 'marker stays on robot right side' rule.
        """
        if math.isinf(self.right_min) or math.isnan(self.right_min):
            return 0.0

        # if the right side is too far away, do not overreact in open space
        if self.right_min > 2.0:
            return 0.0

        err = self.desired_right_dist - self.right_min
        bias = self.right_bias_gain * err
        return clamp(bias, -self.right_bias_max, self.right_bias_max)

    # -------------------------------------------------
    # Motion helpers
    # -------------------------------------------------
    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def publish_manual_drive(self):
        cmd = Twist()
        linear_cmd = self.axis_value(self.AXIS_LINEAR)
        angular_cmd = self.axis_value(self.AXIS_ANGULAR)

        cmd.linear.x = self.manual_linear_scale * linear_cmd
        cmd.angular.z = self.manual_angular_scale * angular_cmd
        self.cmd_pub.publish(cmd)

    def go_to_target(self, target_x, target_y, apply_right_bias=False):
        dx = target_x - self.x
        dy = target_y - self.y
        dist = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        heading_err = normalize_angle(target_yaw - self.yaw)

        if dist < self.wp_tolerance:
            return True

        cmd = Twist()

        if self.front_min < self.obstacle_stop_dist:
            self.get_logger().warn(
                f'Obstacle too close in front: {self.front_min:.2f} m, stopping.'
            )
            self.publish_stop()
            return False

        right_bias = 0.0
        if apply_right_bias:
            right_bias = self.compute_right_side_bias()

        cmd.angular.z = clamp(
            1.6 * heading_err + right_bias,
            -self.max_angular,
            self.max_angular
        )

        if abs(heading_err) > 0.60:
            cmd.linear.x = 0.05
        else:
            cmd.linear.x = min(self.max_linear, 0.10 + 0.15 * dist)

        self.cmd_pub.publish(cmd)
        return False
        
    def build_shape_svg(self, shape_name, color):
        if shape_name == 'circle':
            return f'<circle cx="400" cy="230" r="80" fill="{color}" stroke="black" stroke-width="4"/>'
        elif shape_name == 'triangle':
            return f'<polygon points="400,120 320,300 480,300" fill="{color}" stroke="black" stroke-width="4"/>'
        elif shape_name == 'rectangle':
            return f'<rect x="320" y="150" width="160" height="120" fill="{color}" stroke="black" stroke-width="4"/>'
        elif shape_name == 'cone_right':
            return (
                '<polygon points="620,140 560,320 680,320" '
                'fill="orange" stroke="black" stroke-width="4"/>'
            )
        return ''


    def save_placeholder_svg(self, out_path, title, subtitle, shape_name, color):
        shape_svg = self.build_shape_svg(shape_name, color)

        svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="800" height="500">
      <rect width="100%" height="100%" fill="#f4f4f4"/>
      <text x="40" y="60" font-size="32" font-family="Arial" fill="#222">{title}</text>
      <text x="40" y="105" font-size="22" font-family="Arial" fill="#555">{subtitle}</text>
      <line x1="0" y1="420" x2="800" y2="420" stroke="#999" stroke-width="3"/>
      {shape_svg}
    </svg>'''

        with open(out_path, 'w') as f:
            f.write(svg)

    # -------------------------------------------------
    # Part 2 placeholder hooks
    # -------------------------------------------------
    def capture_marker(self, wp_idx):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'marker_wp{wp_idx + 1}_{timestamp}.svg'
        out_path = os.path.join(self.images_dir, filename)

        self.save_placeholder_svg(
            out_path=out_path,
            title=f'Marker capture for waypoint {wp_idx + 1}',
            subtitle='Simulated placeholder image (marker kept on robot right side concept)',
            shape_name='cone_right',
            color='orange'
        )

        self.generated_images.append(out_path)
        self.get_logger().info(f'[CAPTURE] Placeholder marker image saved: {out_path}')
        return out_path

    def detect_shape_near_waypoint(self, wp_idx):
        shapes = ['circle', 'triangle', 'rectangle']
        colors = ['#4CAF50', '#2196F3', '#9C27B0']

        shape = shapes[wp_idx % len(shapes)]
        color = colors[wp_idx % len(colors)]
        distance = round(1.2 + 0.15 * wp_idx, 2)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'object_wp{wp_idx + 1}_{shape}_{timestamp}.svg'
        out_path = os.path.join(self.images_dir, filename)

        self.save_placeholder_svg(
            out_path=out_path,
            title=f'Object detection near waypoint {wp_idx + 1}',
            subtitle=f'Simulated detected shape: {shape}',
            shape_name=shape,
            color=color
        )

        self.generated_images.append(out_path)

        result = {
            'shape': shape,
            'photo': out_path,
            'distance_to_marker': distance
        }

        self.get_logger().info(
            f"[SEARCH] Placeholder object result: shape={result['shape']}, "
            f"photo={result['photo']}, distance={result['distance_to_marker']:.2f} m"
        )
        return result

    def keep_marker_on_right_placeholder(self):
        # Part 2 placeholder:
        # later this should bias the robot so the cone remains on the robot's right side
        # For now, keep as a documented stub.
        return

    # -------------------------------------------------
    # Weave generation
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

        # perpendicular direction
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
        # Part 2 requires weaving between the 1st and 2nd waypoint
        # This implementation creates intermediate offset points after waypoint 1
        # before waypoint 2. 
        if not self.weave_enabled:
            return False

        if self.current_wp_idx != 1:
            return False

        start_wp = self.waypoints[0]
        end_wp = self.waypoints[1]

        self.weave_points = self.generate_weave_points(
            start_wp,
            end_wp,
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
            'summary_version': 'v3',
            'mission_status': self.mission_status,
            'mission_start_unix': self.mission_start_time,
            'mission_end_unix': mission_end_time,
            'mission_start_iso': datetime.fromtimestamp(self.mission_start_time).isoformat(),
            'mission_end_iso': datetime.fromtimestamp(mission_end_time).isoformat(),
            'duration_sec': round(mission_end_time - self.mission_start_time, 3),
            'returned_home': self.mission_status == 'completed',
            'mode_end': self.mode,
            'weave_enabled': self.weave_enabled,
            'images_dir': self.images_dir,
            'generated_images': self.generated_images,
            'path_plot_exists': os.path.exists(self.path_plot_path),
            'path_plot_file': self.path_plot_path if os.path.exists(self.path_plot_path) else None,
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
        self.get_logger().info('Mission complete. Returned home successfully.')
        self.get_logger().info('================================================')

    # -------------------------------------------------
    # Main control loop
    # -------------------------------------------------
    def control_loop(self):
        if not self.odom_ready or not self.scan_ready:
            self.publish_stop()
            return

        # if /joy is not available yet, keep development moving
        if not self.allow_auto_without_joy:
            if not self.joy_ready:
                self.publish_stop()
                return

            if not self.deadman_pressed:
                self.publish_stop()
                return

        else:
            # development mode: only enforce dead-man if joy is actually connected
            if self.joy_ready and not self.deadman_pressed:
                self.publish_stop()
                return

        if self.mode == 'MANUAL':
            if self.joy_ready:
                self.publish_manual_drive()
            else:
                self.publish_stop()
            return

        # AUTO mode
        if self.state == 'NAVIGATE':
            target = self.waypoints[self.current_wp_idx]
            self.keep_marker_on_right_placeholder()
            reached = self.go_to_target(target[0], target[1], apply_right_bias=True)

            if reached:
                self.publish_stop()
                self.state = 'REACHED'
                self.get_logger().info(
                    f'Reached waypoint {self.current_wp_idx + 1}: {target}'
                )

        elif self.state == 'REACHED':
            self.state = 'CAPTURE'

        elif self.state == 'CAPTURE':
            photo = self.capture_marker(self.current_wp_idx)
            self.last_marker_photo = photo
            self.state = 'SEARCH'

        elif self.state == 'SEARCH':
            result = self.detect_shape_near_waypoint(self.current_wp_idx)

            self.mission_log.append({
                'waypoint_index': self.current_wp_idx + 1,
                'waypoint': self.waypoints[self.current_wp_idx],
                'marker_photo': self.last_marker_photo,
                'shape': result['shape'],
                'object_photo': result['photo'],
                'distance_to_marker': result['distance_to_marker'],
            })

            # after waypoint 1, before waypoint 2, enter weave
            if self.current_wp_idx == 0 and self.weave_enabled:
                self.current_wp_idx += 1
                entered = self.maybe_enter_weave()
                if entered:
                    return

            if self.current_wp_idx < len(self.waypoints) - 1:
                self.current_wp_idx += 1
                self.state = 'NAVIGATE'
                self.get_logger().info(
                    f'Moving to next waypoint {self.current_wp_idx + 1}: '
                    f'{self.waypoints[self.current_wp_idx]}'
                )
            else:
                self.state = 'RETURN_HOME'
                self.get_logger().info('All mission waypoints completed. Returning home.')

        elif self.state == 'WEAVE':
            if self.weave_idx < len(self.weave_points):
                target = self.weave_points[self.weave_idx]
                reached = self.go_to_target(target[0], target[1], apply_right_bias=True)

                if reached:
                    self.publish_stop()
                    self.get_logger().info(
                        f'Reached weave point {self.weave_idx + 1}: {target}'
                    )
                    self.weave_idx += 1
            else:
                self.state = 'NAVIGATE'
                self.get_logger().info(
                    f'WEAVE complete. Continuing to waypoint {self.current_wp_idx + 1}: '
                    f'{self.waypoints[self.current_wp_idx]}'
                )

        elif self.state == 'RETURN_HOME':
            reached_home = self.go_to_target(self.home[0], self.home[1], apply_right_bias=False)
            if reached_home:
                self.publish_stop()
                self.state = 'DONE'
                self.get_logger().info('Reached home position.')

        elif self.state == 'DONE':
            self.publish_stop()
            self.mission_status = 'completed'
            self.print_summary_once()


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.mission_status = 'aborted'
    finally:
        try:
            if rclpy.ok():
                node.publish_stop()
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