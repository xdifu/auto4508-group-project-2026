import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# ==========================================
# 独立模块 1：操纵杆映射器 (纯算法，不依赖ROS底层，极度低耦合)
# 功能：专门负责把手柄的原始数据翻译成速度指令
# ==========================================
class TeleopMapper:
    def __init__(self, axis_linear, axis_angular, max_linear, max_angular):
        self.axis_linear = axis_linear
        self.axis_angular = axis_angular
        self.max_linear = max_linear
        self.max_angular = max_angular

    def get_velocity(self, joy_msg: Joy) -> Twist:
        cmd = Twist()
        # 摇杆数值(-1.0 到 1.0) 乘以 最大速度
        cmd.linear.x = joy_msg.axes[self.axis_linear] * self.max_linear
        cmd.angular.z = joy_msg.axes[self.axis_angular] * self.max_angular
        return cmd

# ==========================================
# 独立模块 2：安全仲裁器 (纯逻辑判断)
# 功能：决定放行哪一条指令
# ==========================================
def arbitrate_velocity(is_auto_mode: bool, dead_man_pressed: bool, auto_twist: Twist, manual_twist: Twist) -> Twist:
    if is_auto_mode:
        if dead_man_pressed:
            return auto_twist  # 安全通过，执行算法速度
        else:
            return Twist()     # 危险！松开死人开关，返回全 0 的急刹车指令
    else:
        return manual_twist    # 手动模式，直接执行手柄推杆速度

# ==========================================
# 核心节点：ROS2 接口层 (高内聚，只负责调度和通信)
# ==========================================
class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_multiplexer')

        # 1. 声明 ROS2 参数 (彻底消除代码里的 Hardcode)
        # 这样可以在命令行或 launch 文件里动态修改这些值
        self.declare_parameter('btn_auto', 0)
        self.declare_parameter('btn_manual', 1)
        self.declare_parameter('btn_deadman', 4)
        self.declare_parameter('axis_linear', 1)
        self.declare_parameter('axis_angular', 0)
        self.declare_parameter('max_speed_linear', 0.5)
        self.declare_parameter('max_speed_angular', 1.0)

        # 读取参数
        self.btn_auto = self.get_parameter('btn_auto').value
        self.btn_manual = self.get_parameter('btn_manual').value
        self.btn_deadman = self.get_parameter('btn_deadman').value

        # 2. 实例化独立的工具类
        self.mapper = TeleopMapper(
            axis_linear=self.get_parameter('axis_linear').value,
            axis_angular=self.get_parameter('axis_angular').value,
            max_linear=self.get_parameter('max_speed_linear').value,
            max_angular=self.get_parameter('max_speed_angular').value
        )

        # 3. 状态变量
        self.is_auto_mode = False 
        self.dead_man_pressed = False
        self.manual_twist = Twist()
        self.auto_twist = Twist()

        # 4. 通信接口
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.auto_sub = self.create_subscription(Twist, '/cmd_vel_auto', self.auto_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 5. 心跳循环
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("🛡️ Modular Safety Node Started (MANUAL MODE).")

    def joy_cb(self, msg: Joy):
        """处理手柄输入，更新状态，并调用辅助模块计算手动速度"""
        # 模式切换
        if msg.buttons[self.btn_auto] == 1 and not self.is_auto_mode:
            self.is_auto_mode = True
            self.get_logger().warn("Switched to AUTO Mode! Hold dead-man switch to move.")
        elif msg.buttons[self.btn_manual] == 1 and self.is_auto_mode:
            self.is_auto_mode = False
            self.get_logger().info("Switched to MANUAL Mode!")

        self.dead_man_pressed = (msg.buttons[self.btn_deadman] == 1)
        
        # 调用工具类：把摇杆数据翻译成 Twist 速度
        self.manual_twist = self.mapper.get_velocity(msg)

    def auto_cb(self, msg: Twist):
        """缓存队友的自动驾驶速度"""
        self.auto_twist = msg

    def control_loop(self):
        """使用独立的逻辑函数进行安全仲裁"""
        final_cmd = arbitrate_velocity(
            self.is_auto_mode, 
            self.dead_man_pressed, 
            self.auto_twist, 
            self.manual_twist
        )
        
        # 增加日志提示：如果触发急刹车，打印警告
        if self.is_auto_mode and not self.dead_man_pressed:
            self.get_logger().error("🛑 EMERGENCY STOP!", throttle_duration_sec=1.0)

        self.cmd_pub.publish(final_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前检查 ROS 网络是否还活着，活着才发刹车
        if rclpy.ok():
            try:
                node.cmd_pub.publish(Twist()) # 强退前急刹
            except Exception:
                pass
        
        node.destroy_node()
        # 只有在还没关闭的情况下，才调用 shutdown
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()