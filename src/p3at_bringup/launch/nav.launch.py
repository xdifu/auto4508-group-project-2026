import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    record = LaunchConfiguration("record")
    rviz = LaunchConfiguration("rviz")
    waypoints_file = LaunchConfiguration("waypoints_file")
    policy_file = LaunchConfiguration("policy_file")

    bringup_share = get_package_share_directory("p3at_bringup")
    description_share = get_package_share_directory("p3at_description")

    nav2_config = os.path.join(bringup_share, "config", "nav2_part2.yaml")
    ekf_local_config = os.path.join(bringup_share, "config", "ekf_local.yaml")
    ekf_global_config = os.path.join(bringup_share, "config", "ekf_global.yaml")
    navsat_config = os.path.join(bringup_share, "config", "navsat_transform.yaml")
    default_waypoints = os.path.join(bringup_share, "config", "waypoints_gps.yaml")
    default_policy = os.path.join(bringup_share, "config", "mission_policy.yaml")
    rviz_config = os.path.join(description_share, "rviz", "nav.rviz")

    cleanup_command = (
        "self=$$; "
        "pgrep -af 'navsat_transform_node|ekf_local_node|ekf_global_node|"
        "planner_server|controller_server|behavior_server|bt_navigator|"
        "nav2_lifecycle_manager/lifecycle_manager|"
        "p3at_bringup/lib/p3at_bringup/mission_orchestrator|"
        "p3at_bringup/lib/p3at_bringup/gps_health_monitor|"
        "p3at_bringup/lib/p3at_bringup/path_recorder|"
        "ros2 bag record' "
        "| awk -v self=\"$self\" '$1 != self {print $1}' | xargs -r kill -TERM; "
        "sleep 1"
    )

    common_nav_params = [nav2_config, {"use_sim_time": use_sim_time}]
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    ekf_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local_node",
        output="screen",
        parameters=[ekf_local_config, {"use_sim_time": use_sim_time}],
        remappings=[
            ("odometry/filtered", "/odometry/filtered/local"),
            ("/odometry/filtered", "/odometry/filtered/local"),
        ],
    )

    ekf_global = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global_node",
        output="screen",
        parameters=[ekf_global_config, {"use_sim_time": use_sim_time}],
        remappings=[
            ("odometry/filtered", "/odometry/filtered/global"),
            ("/odometry/filtered", "/odometry/filtered/global"),
        ],
    )

    navsat_transform = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[navsat_config, {"use_sim_time": use_sim_time}],
        remappings=[
            ("imu/data", "/imu"),
            ("gps/fix", "/fix"),
            ("odometry/filtered", "/odometry/filtered/global"),
            ("odometry/gps", "/odometry/gps"),
            ("gps/filtered", "/gps/filtered"),
        ],
    )

    gps_health_monitor = Node(
        package="p3at_bringup",
        executable="gps_health_monitor",
        name="gps_health_monitor",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=common_nav_params,
        arguments=["--ros-args", "--log-level", "info"],
        remappings=remappings,
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=common_nav_params,
        arguments=["--ros-args", "--log-level", "info"],
        remappings=remappings,
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=common_nav_params,
        arguments=["--ros-args", "--log-level", "info"],
        remappings=remappings,
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=common_nav_params,
        arguments=["--ros-args", "--log-level", "info"],
        remappings=remappings,
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": [
                "controller_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
            ]},
        ],
    )

    mission_orchestrator = Node(
        package="p3at_bringup",
        executable="mission_orchestrator",
        name="mission_orchestrator",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"waypoints_file": waypoints_file},
            {"policy_file": policy_file},
            {"autostart": autostart},
            {"require_safety_topics": False},
        ],
    )

    path_recorder = Node(
        package="p3at_bringup",
        executable="path_recorder",
        name="path_recorder",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"odom_topic": "/odometry/filtered/global"},
            {"output_csv": os.path.join(os.getcwd(), "part2_driven_path.csv")},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
    )

    bag_record = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "/tf",
            "/tf_static",
            "/clock",
            "/odom",
            "/odometry/filtered/local",
            "/odometry/filtered/global",
            "/odometry/gps",
            "/fix",
            "/gps/filtered",
            "/mission/state",
            "/mission/event",
            "/mission/gps_health",
            "/mission/current_waypoint",
            "/scan",
            "/imu",
            "/cmd_vel",
            "/camera/image",
            "/camera/camera_info",
            "-o",
            "part2_demo_run",
        ],
        output="screen",
        condition=IfCondition(record),
    )

    cleanup = ExecuteProcess(
        cmd=["bash", "-lc", cleanup_command],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock."),
        DeclareLaunchArgument("autostart", default_value="true", description="Autostart Nav2 lifecycle nodes."),
        DeclareLaunchArgument("record", default_value="false", description="Record a rosbag2 session."),
        DeclareLaunchArgument("rviz", default_value="false", description="Launch RViz for navigation."),
        DeclareLaunchArgument("waypoints_file", default_value=default_waypoints, description="GPS mission waypoint YAML."),
        DeclareLaunchArgument("policy_file", default_value=default_policy, description="Mission policy YAML."),
        cleanup,
        TimerAction(
            period=1.5,
            actions=[
                ekf_local,
                navsat_transform,
                ekf_global,
                gps_health_monitor,
                planner_server,
                controller_server,
                behavior_server,
                bt_navigator,
                lifecycle_manager,
                mission_orchestrator,
                path_recorder,
                rviz_node,
                bag_record,
            ],
        ),
    ])
