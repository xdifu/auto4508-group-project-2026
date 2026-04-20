from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_port = LaunchConfiguration("robot_port")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    publish_tf = LaunchConfiguration("publish_tf")
    loop_hz = LaunchConfiguration("loop_hz")
    cmd_timeout_sec = LaunchConfiguration("cmd_timeout_sec")
    max_trans_vel_mm_s = LaunchConfiguration("max_trans_vel_mm_s")
    max_rot_vel_deg_s = LaunchConfiguration("max_rot_vel_deg_s")

    aria_node = Node(
        package="aria_node",
        executable="ariaNode",
        name="aria_node",
        output="screen",
        arguments=["-rp", robot_port],
        parameters=[
            {
                "cmd_vel_topic": cmd_vel_topic,
                "odom_topic": odom_topic,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "publish_tf": publish_tf,
                "loop_hz": loop_hz,
                "cmd_timeout_sec": cmd_timeout_sec,
                "max_trans_vel_mm_s": max_trans_vel_mm_s,
                "max_rot_vel_deg_s": max_rot_vel_deg_s,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("publish_tf", default_value="false"),
            DeclareLaunchArgument("loop_hz", default_value="20.0"),
            DeclareLaunchArgument("cmd_timeout_sec", default_value="0.5"),
            DeclareLaunchArgument("max_trans_vel_mm_s", default_value="500.0"),
            DeclareLaunchArgument("max_rot_vel_deg_s", default_value="60.0"),
            aria_node,
        ]
    )
