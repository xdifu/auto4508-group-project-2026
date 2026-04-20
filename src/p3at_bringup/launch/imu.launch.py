import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def _optional_imu_bringup(context):
    package_name = LaunchConfiguration("imu_bringup_package").perform(context).strip()
    launch_name = LaunchConfiguration("imu_bringup_launch").perform(context).strip()
    if not package_name or not launch_name:
        return []
    launch_path = os.path.join(get_package_share_directory(package_name), "launch", launch_name)
    return [
        GroupAction(
            actions=[
                SetRemap(src="/imu/data_raw", dst="/imu/data"),
                SetRemap(src="imu/data_raw", dst="/imu/data"),
                IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_path)),
            ]
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("imu_bringup_package", default_value="phidgets_spatial"),
            DeclareLaunchArgument("imu_bringup_launch", default_value="spatial-launch.py"),
            OpaqueFunction(function=_optional_imu_bringup),
        ]
    )
