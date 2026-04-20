#!/usr/bin/env python3

import math
import csv
import os

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class WaypointController(Node):
    def __init__(self):
        super().__init__('waypoint_controller')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_ready = False

        self.front_min = 999.0
        self.left_front_min = 999.0
        self.left_min = 999.0
        self.scan_ready = False

        self.mode = 'GO_TO_GOAL'
        self.hit_dist = 999.0
        self.last_state = ''

        # ===== multiple waypoints =====
        
        self.waypoints = [
            (0.8, 0.0, 0.0),
            (1.6, 0.3, 0.0),
            (2.6, 0.1, 0.0),
            (3.4, -0.3, 1.57),
        ]
        self.wp_index = 0

        self.goal_x = self.waypoints[0][0]
        self.goal_y = self.waypoints[0][1]
        self.goal_yaw = self.waypoints[0][2]

        # ===== path logging =====
        log_dir = os.path.expanduser('~/Downloads/ros_workshop_ws')
        os.makedirs(log_dir, exist_ok=True)
        self.log_path = os.path.join(log_dir, 'driven_path.csv')

        self.log_file = open(self.log_path, 'w', newline='')
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow([
            'time_sec',
            'x',
            'y',
            'yaw',
            'mode',
            'goal_x',
            'goal_y',
            'front_min',
            'left_front_min',
            'left_min'
        ])

        self.last_log_time = -1.0
        self.log_period = 0.1

        self.timer = self.create_timer(0.1, self.control_loop)

    def set_state(self, state_name: str):
        if self.last_state != state_name:
            self.get_logger().info(state_name)
            self.last_state = state_name

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def goal_distance(self):
        return math.hypot(self.goal_x - self.x, self.goal_y - self.y)

    def goal_heading_error(self):
        desired_yaw = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
        return self.normalize_angle(desired_yaw - self.yaw)

    def final_yaw_error(self):
        return self.normalize_angle(self.goal_yaw - self.yaw)

    def is_final_waypoint(self):
        return self.wp_index == len(self.waypoints) - 1

    def load_current_waypoint(self):
        self.goal_x = self.waypoints[self.wp_index][0]
        self.goal_y = self.waypoints[self.wp_index][1]
        self.goal_yaw = self.waypoints[self.wp_index][2]

    def advance_waypoint(self):
        if self.wp_index < len(self.waypoints) - 1:
            self.wp_index += 1
            self.load_current_waypoint()
            self.mode = 'GO_TO_GOAL'
            self.get_logger().info(
                f'SWITCH_WAYPOINT -> index={self.wp_index}, '
                f'goal=({self.goal_x:.2f}, {self.goal_y:.2f}, yaw={self.goal_yaw:.2f})'
            )
            return True
        return False

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.odom_ready = True

    def scan_callback(self, msg: LaserScan):
        far = msg.range_max if math.isfinite(msg.range_max) and msg.range_max > 0.0 else 999.0

        if not msg.ranges:
            self.scan_ready = False
            self.front_min = far
            self.left_front_min = far
            self.left_min = far
            return

        def sector_min(center_angle, half_width_deg=15):
            vals = []
            half_width = math.radians(half_width_deg)

            for i, r in enumerate(msg.ranges):
                angle = msg.angle_min + i * msg.angle_increment
                if abs(angle - center_angle) <= half_width:
                    if math.isfinite(r) and r > 0.0:
                        vals.append(r)

            return min(vals) if vals else far

        self.front_min = sector_min(0.0, 15)                    # front
        self.left_front_min = sector_min(math.radians(45), 15)  # left_front
        self.left_min = sector_min(math.radians(90), 20)        # left
        self.scan_ready = True

    def log_path_row(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        if self.last_log_time >= 0.0 and (now_sec - self.last_log_time) < self.log_period:
            return

        self.log_writer.writerow([
            round(now_sec, 3),
            round(self.x, 4),
            round(self.y, 4),
            round(self.yaw, 4),
            self.mode,
            round(self.goal_x, 4),
            round(self.goal_y, 4),
            round(self.front_min, 4),
            round(self.left_front_min, 4),
            round(self.left_min, 4),
        ])
        self.log_file.flush()
        self.last_log_time = now_sec

    def control_loop(self):
        if not self.odom_ready or not self.scan_ready:
            return

        self.log_path_row()

        cmd = Twist()

        dist = self.goal_distance()
        heading_err = self.goal_heading_error()
        yaw_err = self.final_yaw_error()

        # =====================================
        # DONE
        # =====================================
        if self.mode == 'DONE':
            self.set_state('GOAL_REACHED')
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        # =====================================
        # final orientation align
        # =====================================
        if self.mode == 'ALIGN_FINAL':
            self.set_state('ALIGN_FINAL')

            if abs(yaw_err) < 0.08:
                self.mode = 'DONE'
                self.set_state('GOAL_REACHED')
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                return

            cmd.linear.x = 0.0
            cmd.angular.z = max(-0.6, min(0.6, 1.5 * yaw_err))
            self.cmd_pub.publish(cmd)
            return

        # =====================================
        # waypoint reached
        # =====================================
        if dist < 0.20:
            if self.is_final_waypoint():
                self.mode = 'ALIGN_FINAL'
                self.get_logger().info(
                    f'FINAL_POSITION_REACHED, yaw_err={yaw_err:.3f}'
                )
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                return
            else:
                self.get_logger().info(
                    f'WAYPOINT_REACHED index={self.wp_index}, '
                    f'pos=({self.goal_x:.2f}, {self.goal_y:.2f})'
                )
                self.advance_waypoint()
                return

        # =====================================
        # GO TO GOAL
        # =====================================
        if self.mode == 'GO_TO_GOAL':
            self.set_state('GO_TO_GOAL')

            # obstacle detected -> enter wall follow
            if self.front_min < 0.55:
                self.mode = 'WALL_FOLLOW'
                self.hit_dist = dist
                self.get_logger().info(
                    f'ENTER_WALL_FOLLOW, front={self.front_min:.3f}, dist={dist:.3f}'
                )
                return

            # heading first
            if abs(heading_err) > 0.20:
                cmd.linear.x = 0.0
                cmd.angular.z = max(-0.8, min(0.8, 1.5 * heading_err))
            else:
                cmd.linear.x = 0.18
                cmd.angular.z = max(-0.5, min(0.5, 1.0 * heading_err))

            self.cmd_pub.publish(cmd)
            return

        # =====================================
        # WALL FOLLOW
        # =====================================
        if self.mode == 'WALL_FOLLOW':
            self.set_state('WALL_FOLLOW')

            
            if dist < (self.hit_dist - 0.15) and self.front_min > 0.80:
                self.mode = 'GO_TO_GOAL'
                self.get_logger().info(
                    f'LEAVE_WALL_FOLLOW, front={self.front_min:.3f}, dist={dist:.3f}'
                )
                return

            desired_left = 0.55

            dist_err = self.left_min - desired_left
            wall_heading_err = self.left_front_min - self.left_min

            if self.front_min < 0.38:
                cmd.linear.x = -0.02
                cmd.angular.z = -0.90

            elif self.front_min < 0.60 or self.left_front_min < 0.42:
                cmd.linear.x = 0.02
                cmd.angular.z = -0.75

            else:
                u = 1.2 * dist_err + 2.2 * wall_heading_err

                if abs(u) > 0.35:
                    cmd.linear.x = 0.06
                else:
                    cmd.linear.x = 0.10

                cmd.angular.z = max(-0.55, min(0.55, u))

            self.cmd_pub.publish(cmd)
            return

    def destroy_node(self):
        try:
            if hasattr(self, 'log_file') and self.log_file:
                self.log_file.close()
        except Exception:
            pass

        try:
            if rclpy.ok():
                stop = Twist()
                self.cmd_pub.publish(stop)
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
