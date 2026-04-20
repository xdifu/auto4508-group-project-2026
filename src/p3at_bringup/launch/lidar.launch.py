import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _optional_lidar_bringup(context):
    package_name = LaunchConfiguration("lidar_bringup_package").perform(context).strip()
    launch_name = LaunchConfiguration("lidar_bringup_launch").perform(context).strip()
    if not package_name or not launch_name:
        return []
    hostname = LaunchConfiguration("lidar_hostname").perform(context).strip()
    frame_id = LaunchConfiguration("lidar_frame_id").perform(context).strip()
    if package_name == "sick_scan_xd":
        node_name = launch_name.replace(".launch.py", "").replace(".launch", "")
        return [
            Node(
                package="sick_scan_xd",
                executable="sick_generic_caller",
                output="screen",
                name=node_name,
                parameters=[
                    {
                        "scanner_type": "sick_tim_7xx",
                        "nodename": node_name,
                        "min_ang": -2.35619449,
                        "max_ang": 2.35619449,
                        "use_binary_protocol": True,
                        "range_min": 0.0,
                        "range_max": 100.0,
                        "range_filter_handling": 0,
                        "intensity": True,
                        "hostname": hostname or "192.168.0.1",
                        "cloud_topic": "cloud",
                        "laserscan_topic": "scan",
                        "frame_id": frame_id or "laser_frame",
                        "port": "2112",
                        "timelimit": 5,
                        "sw_pll_only_publish": True,
                        "use_generation_timestamp": True,
                        "start_services": True,
                        "activate_lferec": True,
                        "activate_lidoutputstate": True,
                        "activate_lidinputstate": True,
                        "active_field_set": -1,
                        "field_set_selection_method": -1,
                        "min_intensity": 0.0,
                        "scandatacfg_timingflag": -1,
                        "add_transform_xyz_rpy": "0,0,0,0,0,0",
                        "add_transform_check_dynamic_updates": False,
                        "message_monitoring_enabled": True,
                        "read_timeout_millisec_default": 5000,
                        "read_timeout_millisec_startup": 120000,
                        "read_timeout_millisec_kill_node": 150000,
                        "user_level": 3,
                        "user_level_password": "F4724744",
                        "ros_qos": -1,
                        "tf_base_frame_id": "base_link",
                        "tf_base_lidar_xyz_rpy": "0,0,0,0,0,0",
                        "tf_publish_rate": 0.0,
                        "tick_to_timestamp_mode": 0,
                    }
                ],
            )
        ]

    launch_path = os.path.join(get_package_share_directory(package_name), "launch", launch_name)
    launch_arguments = {}
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
