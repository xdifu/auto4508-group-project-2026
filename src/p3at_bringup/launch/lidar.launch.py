import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _configured_ipv4_addresses():
    configured = set()
    try:
        result = subprocess.run(
            ["ip", "-o", "-4", "addr", "show"],
            check=True,
            capture_output=True,
            text=True,
        )
        configured.update(
            match.group(1)
            for match in re.finditer(r"\binet\s+(\d+\.\d+\.\d+\.\d+)/\d+", result.stdout)
        )
    except (FileNotFoundError, subprocess.CalledProcessError):
        pass

    if configured:
        return configured

    try:
        result = subprocess.run(
            ["hostname", "-I"],
            check=True,
            capture_output=True,
            text=True,
        )
    except (FileNotFoundError, subprocess.CalledProcessError):
        return set()

    configured.update(re.findall(r"\b\d+\.\d+\.\d+\.\d+\b", result.stdout))
    return configured


def _build_sick_node(context):
    hostname = LaunchConfiguration("lidar_hostname").perform(context).strip()
    frame_id = LaunchConfiguration("lidar_frame_id").perform(context).strip()
    output_topic = LaunchConfiguration("lidar_output_topic").perform(context).strip()
    node_name = "sick_tim_7xx"
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
                    "laserscan_topic": output_topic or "scan",
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


def _build_lakibeam_node(context):
    host_ip = LaunchConfiguration("lidar_host_ip").perform(context).strip()
    if host_ip and host_ip != "0.0.0.0":
        configured_ips = _configured_ipv4_addresses()
        if host_ip not in configured_ips:
            raise RuntimeError(
                f"Lakibeam host IP {host_ip} is not configured on any local interface. "
                f"Configure {host_ip}/24 on the robot NIC for RJ45 mode, or set "
                f"lidar_host_ip:=0.0.0.0 / USB-C parameters before starting."
            )

    return [
        Node(
            package="lakibeam1",
            executable="lakibeam1_scan_node",
            output="screen",
            name="richbeam_lidar_node0",
            parameters=[
                {
                    "frame_id": LaunchConfiguration("lidar_frame_id"),
                    "output_topic": LaunchConfiguration("lidar_output_topic"),
                    "inverted": LaunchConfiguration("lidar_inverted"),
                    "hostip": LaunchConfiguration("lidar_host_ip"),
                    "sensorip": LaunchConfiguration("lidar_sensor_ip"),
                    "port": LaunchConfiguration("lidar_udp_port"),
                    "angle_offset": LaunchConfiguration("lidar_angle_offset"),
                    "apply_sensor_config": LaunchConfiguration("lidar_apply_sensor_config"),
                    "scanfreq": LaunchConfiguration("lidar_scanfreq"),
                    "filter": LaunchConfiguration("lidar_filter_level"),
                    "laser_enable": LaunchConfiguration("lidar_laser_enable"),
                    "scan_range_start": LaunchConfiguration("lidar_scan_start_deg"),
                    "scan_range_stop": LaunchConfiguration("lidar_scan_stop_deg"),
                }
            ],
        )
    ]


def _build_custom_include(context):
    package_name = LaunchConfiguration("lidar_bringup_package").perform(context).strip()
    launch_name = LaunchConfiguration("lidar_bringup_launch").perform(context).strip()
    if not package_name or not launch_name:
        return []

    hostname = LaunchConfiguration("lidar_hostname").perform(context).strip()
    frame_id = LaunchConfiguration("lidar_frame_id").perform(context).strip()
    output_topic = LaunchConfiguration("lidar_output_topic").perform(context).strip()
    launch_arguments = {}
    if hostname:
        launch_arguments["hostname"] = hostname
    if frame_id:
        launch_arguments["frame_id"] = frame_id
    if output_topic:
        launch_arguments["output_topic"] = output_topic
    launch_path = os.path.join(get_package_share_directory(package_name), "launch", launch_name)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
            launch_arguments=launch_arguments.items(),
        )
    ]


def _optional_lidar_bringup(context):
    driver = LaunchConfiguration("lidar_driver").perform(context).strip().lower()
    if driver == "lakibeam":
        return _build_lakibeam_node(context)
    if driver == "sick":
        return _build_sick_node(context)
    if driver == "custom":
        return _build_custom_include(context)
    raise RuntimeError(f"Unsupported lidar_driver '{driver}'. Expected lakibeam, sick, or custom.")


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("lidar_driver", default_value="lakibeam"),
            DeclareLaunchArgument("lidar_host_ip", default_value="192.168.198.50"),
            DeclareLaunchArgument("lidar_sensor_ip", default_value="192.168.198.2"),
            DeclareLaunchArgument("lidar_udp_port", default_value="2368"),
            DeclareLaunchArgument("lidar_frame_id", default_value="laser_frame"),
            DeclareLaunchArgument("lidar_output_topic", default_value="scan"),
            DeclareLaunchArgument("lidar_inverted", default_value="false"),
            DeclareLaunchArgument("lidar_angle_offset", default_value="0"),
            DeclareLaunchArgument("lidar_apply_sensor_config", default_value="false"),
            DeclareLaunchArgument("lidar_scanfreq", default_value="30"),
            DeclareLaunchArgument("lidar_filter_level", default_value="3"),
            DeclareLaunchArgument("lidar_laser_enable", default_value="true"),
            DeclareLaunchArgument("lidar_scan_start_deg", default_value="45"),
            DeclareLaunchArgument("lidar_scan_stop_deg", default_value="315"),
            DeclareLaunchArgument("lidar_bringup_launch", default_value="sick_tim_7xx.launch.py"),
            DeclareLaunchArgument("lidar_bringup_package", default_value=""),
            DeclareLaunchArgument("lidar_hostname", default_value="192.168.0.1"),
            OpaqueFunction(function=_optional_lidar_bringup),
        ]
    )
