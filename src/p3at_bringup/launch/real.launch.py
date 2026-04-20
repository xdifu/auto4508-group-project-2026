import os
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    run_id_default = datetime.now().strftime("%Y%m%d_%H%M%S")
    cwd = os.getcwd()

    bringup_share = get_package_share_directory("p3at_bringup")
    auto_share = get_package_share_directory("auto4508_project")
    vision_share = get_package_share_directory("p3at_vision")
    description_share = get_package_share_directory("p3at_description")

    xacro_file = os.path.join(description_share, "urdf", "pioneer.urdf.xacro")
    robot_description = Command([FindExecutable(name="xacro"), " ", xacro_file])

    run_id = LaunchConfiguration("run_id")
    artifacts_root = LaunchConfiguration("artifacts_root")
    waypoints_file = LaunchConfiguration("waypoints_file")
    policy_file = LaunchConfiguration("policy_file")
    autostart = LaunchConfiguration("autostart")
    record = LaunchConfiguration("record")
    rviz = LaunchConfiguration("rviz")
    startup_delay_sec = LaunchConfiguration("startup_delay_sec")

    nav_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "nav.launch.py")),
        launch_arguments={
            "use_sim_time": "false",
            "autostart": autostart,
            "rviz": rviz,
            "record": record,
            "waypoints_file": waypoints_file,
            "policy_file": policy_file,
            "run_id": run_id,
            "artifacts_root": artifacts_root,
            "require_safety_topics": "true",
        }.items(),
    )

    aria_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "aria_base.launch.py")),
        launch_arguments={
            "robot_port": LaunchConfiguration("robot_port"),
            "cmd_vel_topic": "/cmd_vel",
            "odom_topic": "/odom",
            "odom_frame": "odom",
            "base_frame": "base_link",
            "publish_tf": "false",
            "loop_hz": "20.0",
            "cmd_timeout_sec": "0.5",
            "max_trans_vel_mm_s": "500.0",
            "max_rot_vel_deg_s": "60.0",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"robot_description": robot_description},
        ],
    )

    gps_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "gps.launch.py")),
        launch_arguments={
            "gps_port": LaunchConfiguration("gps_port"),
            "gps_baud": LaunchConfiguration("gps_baud"),
            "gps_frame_id": "gps_link",
        }.items(),
    )

    imu_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "imu.launch.py")),
        launch_arguments={
            "imu_bringup_package": LaunchConfiguration("imu_bringup_package"),
            "imu_bringup_launch": LaunchConfiguration("imu_bringup_launch"),
        }.items(),
    )

    lidar_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "lidar.launch.py")),
        launch_arguments={
            "lidar_bringup_package": LaunchConfiguration("lidar_bringup_package"),
            "lidar_bringup_launch": LaunchConfiguration("lidar_bringup_launch"),
            "lidar_hostname": LaunchConfiguration("lidar_hostname"),
            "lidar_frame_id": LaunchConfiguration("lidar_frame_id"),
        }.items(),
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        output="screen",
        parameters=[os.path.join(auto_share, "config", "ps_teleop.yaml")],
        remappings=[("cmd_vel", "/cmd_vel_manual")],
    )

    safety_node = Node(
        package="auto4508_project",
        executable="safety_node",
        name="safety_node",
        output="screen",
        parameters=[{"run_id": run_id}],
    )

    logger_node = Node(
        package="auto4508_project",
        executable="logger_node",
        name="logger_node",
        output="screen",
        parameters=[
            {
                "run_id": run_id,
                "artifacts_root": artifacts_root,
            }
        ],
    )

    vision_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(vision_share, "launch", "vision.launch.py")),
        launch_arguments={
            "run_id": run_id,
            "save_dir": PathJoinSubstitution([artifacts_root, "photos"]),
            "rgb_topic": "/camera/camera/color/image_raw",
            "depth_topic": "/camera/camera/depth/image_rect_raw",
            "camera_info_topic": "/camera/camera/color/camera_info",
            "navigation_topic": "/mission/navigation",
            "vision_topic": "/mission/vision",
            "use_depth": "true",
            "save_raw": "true",
            "save_annotated": "true",
        }.items(),
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[os.path.join(bringup_share, "config", "slam_toolbox.yaml"), {"use_sim_time": False}],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver",
        output="screen",
        parameters=[{"save_map_timeout": 5.0, "free_thresh_default": 0.25, "occupied_thresh_default": 0.65}],
    )

    map_saver_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_saver",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"autostart": True},
            {"node_names": ["map_saver"]},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "run_id",
                default_value=run_id_default,
                description="Artifact run identifier",
            ),
            DeclareLaunchArgument(
                "artifacts_root",
                default_value=str(Path(cwd) / "artifacts" / "part2_runs" / run_id_default),
                description="Artifact root directory for this run",
            ),
            DeclareLaunchArgument(
                "waypoints_file",
                default_value=os.path.join(bringup_share, "config", "waypoints_gps.yaml"),
            ),
            DeclareLaunchArgument(
                "policy_file",
                default_value=os.path.join(bringup_share, "config", "mission_policy.yaml"),
            ),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("record", default_value="false"),
            DeclareLaunchArgument(
                "startup_delay_sec",
                default_value="12.0",
                description="Delay before starting non-base subsystems to give the Pioneer serial link time to connect",
            ),
            DeclareLaunchArgument("robot_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("gps_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("gps_baud", default_value="9600"),
            DeclareLaunchArgument("imu_bringup_package", default_value="phidgets_spatial"),
            DeclareLaunchArgument("imu_bringup_launch", default_value="spatial-launch.py"),
            DeclareLaunchArgument("lidar_bringup_package", default_value="sick_scan_xd"),
            DeclareLaunchArgument("lidar_bringup_launch", default_value="sick_tim_7xx.launch.py"),
            DeclareLaunchArgument("lidar_hostname", default_value="192.168.0.1"),
            DeclareLaunchArgument("lidar_frame_id", default_value="laser_frame"),
            aria_stack,
            TimerAction(
                period=startup_delay_sec,
                actions=[
                    robot_state_publisher,
                    gps_stack,
                    imu_stack,
                    lidar_stack,
                    joy_node,
                    teleop_node,
                    safety_node,
                    slam_toolbox,
                    map_saver,
                    map_saver_lifecycle,
                    logger_node,
                    vision_stack,
                    nav_stack,
                ],
            ),
        ]
    )
