import json
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String


def zero_twist() -> Twist:
    return Twist()


class SafetyNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_node")

        self.declare_parameter("run_id", "part2_run")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("cmd_vel_nav_topic", "/cmd_vel_nav")
        self.declare_parameter("cmd_vel_manual_topic", "/cmd_vel_manual")
        self.declare_parameter("cmd_vel_out_topic", "/cmd_vel")
        self.declare_parameter("btn_auto", 0)   # X
        self.declare_parameter("btn_manual", 1)  # O
        self.declare_parameter("btn_estop", 2)   # Square
        self.declare_parameter("deadman_axis", 5)
        self.declare_parameter("deadman_axis_threshold", 0.0)
        self.declare_parameter("deadman_axis_inverted", True)
        self.declare_parameter("joy_timeout_sec", 0.25)
        self.declare_parameter("heartbeat_hz", 2.0)
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("estop_clear_hold_sec", 1.0)

        self.run_id = str(self.get_parameter("run_id").value)
        self.joy_topic = str(self.get_parameter("joy_topic").value)
        self.cmd_vel_nav_topic = str(self.get_parameter("cmd_vel_nav_topic").value)
        self.cmd_vel_manual_topic = str(self.get_parameter("cmd_vel_manual_topic").value)
        self.cmd_vel_out_topic = str(self.get_parameter("cmd_vel_out_topic").value)
        self.btn_auto = int(self.get_parameter("btn_auto").value)
        self.btn_manual = int(self.get_parameter("btn_manual").value)
        self.btn_estop = int(self.get_parameter("btn_estop").value)
        self.deadman_axis = int(self.get_parameter("deadman_axis").value)
        self.deadman_axis_threshold = float(self.get_parameter("deadman_axis_threshold").value)
        self.deadman_axis_inverted = bool(self.get_parameter("deadman_axis_inverted").value)
        self.joy_timeout_sec = float(self.get_parameter("joy_timeout_sec").value)
        self.estop_clear_hold_sec = float(self.get_parameter("estop_clear_hold_sec").value)

        self.mode = "MANUAL"
        self.deadman_pressed = False
        self.estop = False
        self.joy_connected = False
        self.last_joy_sec = 0.0
        self.clear_hold_start: Optional[float] = None
        self.estop_clear_armed = False
        self.prev_buttons = []
        self.nav_twist = zero_twist()
        self.manual_twist = zero_twist()
        self.last_cmd = zero_twist()
        self.last_joy_timeout_state = False

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_out_topic, 10)
        self.safety_pub = self.create_publisher(String, "/mission/safety", 10)
        self.auto_pub = self.create_publisher(Bool, "/safety/automated_enabled", 10)
        self.deadman_pub = self.create_publisher(Bool, "/safety/deadman_pressed", 10)
        self.estop_pub = self.create_publisher(Bool, "/safety/estop", 10)

        self.create_subscription(Joy, self.joy_topic, self.joy_cb, 20)
        self.create_subscription(Twist, self.cmd_vel_nav_topic, self.nav_cb, 20)
        self.create_subscription(Twist, self.cmd_vel_manual_topic, self.manual_cb, 20)

        self.control_timer = self.create_timer(1.0 / max(5.0, float(self.get_parameter("control_hz").value)), self.control_loop)
        self.heartbeat_timer = self.create_timer(1.0 / max(0.5, float(self.get_parameter("heartbeat_hz").value)), self.publish_heartbeat)

        self.get_logger().info(
            f"Safety node ready. joy={self.joy_topic} nav={self.cmd_vel_nav_topic} manual={self.cmd_vel_manual_topic}"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def nav_cb(self, msg: Twist) -> None:
        self.nav_twist = msg

    def manual_cb(self, msg: Twist) -> None:
        self.manual_twist = msg

    def joy_cb(self, msg: Joy) -> None:
        now = self.now_sec()
        self.last_joy_sec = now
        if not self.joy_connected:
            self.joy_connected = True
            self.publish_event("joy_recovered")
        self.last_joy_timeout_state = False

        buttons = list(msg.buttons)
        rising_auto = self._rising(buttons, self.btn_auto)
        rising_manual = self._rising(buttons, self.btn_manual)
        rising_estop = self._rising(buttons, self.btn_estop)
        square_pressed = self._pressed(buttons, self.btn_estop)

        if rising_auto and self.mode != "AUTO":
            self.mode = "AUTO"
            self.publish_event("mode_change")
        elif rising_manual and self.mode != "MANUAL":
            self.mode = "MANUAL"
            self.publish_event("mode_change")

        previous_deadman = self.deadman_pressed
        self.deadman_pressed = self._deadman_from_axes(msg)
        if self.deadman_pressed != previous_deadman:
            self.publish_event("deadman_change")

        if rising_estop and not self.estop:
            self.estop = True
            self.clear_hold_start = None
            self.estop_clear_armed = False
            self.publish_event("estop_change")
        elif self.estop:
            if not square_pressed:
                self.clear_hold_start = None
                self.estop_clear_armed = True
            elif self.estop_clear_armed:
                if (not self.deadman_pressed) and self._speed(self.last_cmd) < 0.02:
                    if self.clear_hold_start is None:
                        self.clear_hold_start = now
                    elif now - self.clear_hold_start >= self.estop_clear_hold_sec:
                        self.estop = False
                        self.clear_hold_start = None
                        self.estop_clear_armed = False
                        self.publish_event("estop_change")
                else:
                    self.clear_hold_start = None
        else:
            self.clear_hold_start = None

        self.prev_buttons = buttons

    def _speed(self, twist: Twist) -> float:
        return math.hypot(float(twist.linear.x), float(twist.angular.z))

    def _pressed(self, buttons, index: int) -> bool:
        return 0 <= index < len(buttons) and buttons[index] == 1

    def _rising(self, buttons, index: int) -> bool:
        prev = self.prev_buttons[index] if 0 <= index < len(self.prev_buttons) else 0
        current = buttons[index] if 0 <= index < len(buttons) else 0
        return current == 1 and prev == 0

    def _deadman_from_axes(self, joy_msg: Joy) -> bool:
        if 0 <= self.deadman_axis < len(joy_msg.axes):
            value = float(joy_msg.axes[self.deadman_axis])
            return value < self.deadman_axis_threshold if self.deadman_axis_inverted else value > self.deadman_axis_threshold
        return False

    def joy_timed_out(self) -> bool:
        if self.last_joy_sec <= 0.0:
            return True
        return (self.now_sec() - self.last_joy_sec) > self.joy_timeout_sec

    def arbitrate(self) -> Twist:
        if self.estop or not self.joy_connected or self.joy_timed_out() or not self.deadman_pressed:
            return zero_twist()
        return self.nav_twist if self.mode == "AUTO" else self.manual_twist

    def publish_state_topics(self) -> None:
        self.auto_pub.publish(Bool(data=self.mode == "AUTO"))
        self.deadman_pub.publish(Bool(data=self.deadman_pressed))
        self.estop_pub.publish(Bool(data=self.estop))

    def publish_event(self, event: str) -> None:
        payload = {
            "run_id": self.run_id,
            "mode": self.mode,
            "deadman_pressed": self.deadman_pressed,
            "estop": self.estop,
            "joy_connected": self.joy_connected and not self.joy_timed_out(),
            "event": event,
            "source": "gamepad",
            "ts": round(self.now_sec(), 3),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.safety_pub.publish(msg)
        self.publish_state_topics()

    def publish_heartbeat(self) -> None:
        self.publish_event("heartbeat")

    def control_loop(self) -> None:
        timed_out = self.joy_timed_out()
        if timed_out and not self.last_joy_timeout_state:
            self.last_joy_timeout_state = True
            if self.deadman_pressed:
                self.deadman_pressed = False
                self.publish_event("deadman_change")
            self.joy_connected = False
            self.publish_event("joy_timeout")
        self.last_cmd = self.arbitrate()
        self.cmd_pub.publish(self.last_cmd)
        self.publish_state_topics()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(zero_twist())
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
