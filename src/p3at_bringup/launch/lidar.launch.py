import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _optional_lidar_bringup(context):
    package_name = LaunchConfiguration("lidar_bringup_package").perform(context).strip()
    launch_name = LaunchConfiguration("lidar_bringup_launch").perform(context).strip()
    if not package_name or not launch_name:
        return []
    launch_path = os.path.join(get_package_share_directory(package_name), "launch", launch_name)
    launch_arguments = {}
    hostname = LaunchConfiguration("lidar_hostname").perform(context).strip()
    frame_id = LaunchConfiguration("lidar_frame_id").perform(context).strip()
    if hostname:
        launch_arguments["hostname"] = hostname
    if frame_id:
        launch_arguments["frame_id"] = frame_id
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
            launch_arguments=launch_arguments.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("lidar_bringup_package", default_value="sick_scan_xd"),
            DeclareLaunchArgument("lidar_bringup_launch", default_value="sick_tim_7xx.launch.py"),
            DeclareLaunchArgument("lidar_hostname", default_value="192.168.0.1"),
            DeclareLaunchArgument("lidar_frame_id", default_value="laser_frame"),
            OpaqueFunction(function=_optional_lidar_bringup),
        ]
    )
