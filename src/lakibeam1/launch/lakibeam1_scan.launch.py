from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("frame_id", default_value="laser_frame"),
            DeclareLaunchArgument("output_topic", default_value="scan"),
            DeclareLaunchArgument("inverted", default_value="false"),
            DeclareLaunchArgument("hostip", default_value="0.0.0.0"),
            DeclareLaunchArgument("sensorip", default_value="192.168.198.2"),
            DeclareLaunchArgument("port", default_value="2368"),
            DeclareLaunchArgument("angle_offset", default_value="0"),
            DeclareLaunchArgument("apply_sensor_config", default_value="false"),
            DeclareLaunchArgument("scanfreq", default_value="30"),
            DeclareLaunchArgument("filter", default_value="3"),
            DeclareLaunchArgument("laser_enable", default_value="true"),
            DeclareLaunchArgument("scan_range_start", default_value="45"),
            DeclareLaunchArgument("scan_range_stop", default_value="315"),
            Node(
                package="lakibeam1",
                executable="lakibeam1_scan_node",
                name="richbeam_lidar_node0",
                output="screen",
                parameters=[
                    {
                        "frame_id": LaunchConfiguration("frame_id"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "inverted": LaunchConfiguration("inverted"),
                        "hostip": LaunchConfiguration("hostip"),
                        "sensorip": LaunchConfiguration("sensorip"),
                        "port": LaunchConfiguration("port"),
                        "angle_offset": LaunchConfiguration("angle_offset"),
                        "apply_sensor_config": LaunchConfiguration("apply_sensor_config"),
                        "scanfreq": LaunchConfiguration("scanfreq"),
                        "filter": LaunchConfiguration("filter"),
                        "laser_enable": LaunchConfiguration("laser_enable"),
                        "scan_range_start": LaunchConfiguration("scan_range_start"),
                        "scan_range_stop": LaunchConfiguration("scan_range_stop"),
                    }
                ],
            ),
        ]
    )
