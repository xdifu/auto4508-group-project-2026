import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("p3at_bringup")
    nav_launch = os.path.join(bringup_share, "launch", "nav.launch.py")
    default_waypoints = os.path.join(bringup_share, "config", "waypoints_gps.yaml")
    default_policy = os.path.join(bringup_share, "config", "mission_policy.yaml")

    waypoints_file = LaunchConfiguration("waypoints_file")
    policy_file = LaunchConfiguration("policy_file")
    autostart = LaunchConfiguration("autostart")
    rviz = LaunchConfiguration("rviz")
    record = LaunchConfiguration("record")

    nav_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch),
        launch_arguments={
            "use_sim_time": "false",
            "autostart": autostart,
            "rviz": rviz,
            "record": record,
            "waypoints_file": waypoints_file,
            "policy_file": policy_file,
        }.items(),
    )

    # Hardware driver nodes (GPS/IMU/Lidar/Camera/base) should be launched by the platform bringup.
    return LaunchDescription([
        DeclareLaunchArgument("waypoints_file", default_value=default_waypoints, description="Real-world waypoint YAML."),
        DeclareLaunchArgument("policy_file", default_value=default_policy, description="Mission policy YAML."),
        DeclareLaunchArgument("autostart", default_value="true", description="Autostart mission stack."),
        DeclareLaunchArgument("rviz", default_value="false", description="Launch RViz."),
        DeclareLaunchArgument("record", default_value="false", description="Record rosbag2 topics."),
        nav_stack,
    ])
