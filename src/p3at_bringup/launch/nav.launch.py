import os
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    run_id_default = datetime.now().strftime("%Y%m%d_%H%M%S")
    cwd = os.getcwd()

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    record = LaunchConfiguration("record")
    rviz = LaunchConfiguration("rviz")
    waypoints_file = LaunchConfiguration("waypoints_file")
    policy_file = LaunchConfiguration("policy_file")
    run_id = LaunchConfiguration("run_id")
    artifacts_root = LaunchConfiguration("artifacts_root")
    require_safety_topics = LaunchConfiguration("require_safety_topics")

    bringup_share = get_package_share_directory("p3at_bringup")
    description_share = get_package_share_directory("p3at_description")
    nav2_config = os.path.join(bringup_share, "config", "nav2_part2.yaml")
    ekf_local_config = os.path.join(bringup_share, "config", "ekf_local.yaml")
    ekf_global_config = os.path.join(bringup_share, "config", "ekf_global.yaml")
    navsat_config = os.path.join(bringup_share, "config", "navsat_transform.yaml")
    default_waypoints = os.path.join(bringup_share, "config", "waypoints_gps.yaml")
    default_policy = os.path.join(bringup_share, "config", "mission_policy.yaml")
    bt_xml = os.path.join(bringup_share, "config", "part2_nav_to_pose.xml")
    rviz_config = os.path.join(description_share, "rviz", "nav.rviz")

    common_nav_params = [
        nav2_config,
        {
            "use_sim_time": use_sim_time,
            "default_nav_to_pose_bt_xml": bt_xml,
        },
    ]
    nav_remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]
    cmd_vel_raw_remap = nav_remappings + [("cmd_vel", "/cmd_vel_nav_raw")]

    ekf_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local_node",
        output="screen",
        parameters=[ekf_local_config, {"use_sim_time": use_sim_time}],
        remappings=[("odometry/filtered", "/odometry/filtered/local"), ("/odometry/filtered", "/odometry/filtered/local")],
    )

    ekf_global = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global_node",
        output="screen",
        parameters=[ekf_global_config, {"use_sim_time": use_sim_time}],
        remappings=[("odometry/filtered", "/odometry/filtered/global"), ("/odometry/filtered", "/odometry/filtered/global")],
    )

    navsat_transform = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[navsat_config, {"use_sim_time": use_sim_time}],
        remappings=[
            ("imu/data", "/imu/data"),
            ("gps/fix", "/fix"),
            ("odometry/filtered", "/odometry/filtered/local"),
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

    scan_sanitizer = Node(
        package="p3at_bringup",
        executable="scan_sanitizer",
        name="scan_sanitizer",
        output="screen",
        parameters=[
            {
                "input_topic": "/scan",
                "output_topic": "/scan_nav",
            }
        ],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=common_nav_params,
        remappings=nav_remappings,
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=common_nav_params,
        remappings=cmd_vel_raw_remap,
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=common_nav_params,
        remappings=cmd_vel_raw_remap,
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=common_nav_params,
        remappings=nav_remappings,
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=common_nav_params,
        remappings=[
            ("cmd_vel", "/cmd_vel_nav_raw"),
            ("cmd_vel_smoothed", "/cmd_vel_nav_pre_collision"),
            ("odom", "/odometry/filtered/local"),
        ],
    )

    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=common_nav_params,
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "velocity_smoother",
                    "collision_monitor",
                ]
            },
        ],
    )

    supervisor = Node(
        package="p3at_bringup",
        executable="part2_supervisor",
        name="part2_supervisor",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "waypoints_file": waypoints_file,
                "policy_file": policy_file,
                "autostart": autostart,
                "require_safety_topics": require_safety_topics,
                "run_id": run_id,
                "artifacts_root": artifacts_root,
            }
        ],
    )

    path_recorder = Node(
        package="p3at_bringup",
        executable="path_recorder",
        name="path_recorder",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "odom_topic": "/odometry/filtered/global",
                "path_topic": "/driven_path",
                "output_csv": PathJoinSubstitution([artifacts_root, "summary", "path.csv"]),
            }
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
            "/odometry/filtered/local",
            "/odometry/filtered/global",
            "/fix",
            "/scan",
            "/imu/data",
            "/cmd_vel",
            "/cmd_vel_nav",
            "/cmd_vel_manual",
            "/mission/navigation",
            "/mission/vision",
            "/mission/safety",
            "/driven_path",
            "/camera/camera/color/image_raw",
            "/camera/camera/color/camera_info",
            "/camera/camera/depth/image_rect_raw",
            "-o",
            PathJoinSubstitution([artifacts_root, "bags", "part2_demo_run"]),
        ],
        output="screen",
        condition=IfCondition(record),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("record", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("waypoints_file", default_value=default_waypoints),
            DeclareLaunchArgument("policy_file", default_value=default_policy),
            DeclareLaunchArgument("run_id", default_value=run_id_default),
            DeclareLaunchArgument("artifacts_root", default_value=str(Path(cwd) / "artifacts" / "part2_runs" / run_id_default)),
            DeclareLaunchArgument("require_safety_topics", default_value="true"),
            TimerAction(
                period=1.5,
                actions=[
                    ekf_local,
                    navsat_transform,
                    ekf_global,
                    gps_health_monitor,
                    scan_sanitizer,
                    planner_server,
                    controller_server,
                    behavior_server,
                    bt_navigator,
                    velocity_smoother,
                    collision_monitor,
                    lifecycle_manager,
                    supervisor,
                    path_recorder,
                    rviz_node,
                    bag_record,
                ],
            ),
        ]
    )
