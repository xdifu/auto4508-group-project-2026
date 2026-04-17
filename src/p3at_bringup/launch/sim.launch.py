import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    teleop = LaunchConfiguration("teleop")
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")
    waypoints_file = LaunchConfiguration("waypoints_file")
    gps_noise_h = LaunchConfiguration("gps_noise_h")
    gps_noise_v = LaunchConfiguration("gps_noise_v")

    description_share = get_package_share_directory("p3at_description")
    bringup_share = get_package_share_directory("p3at_bringup")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    xacro_file = os.path.join(description_share, "urdf", "pioneer.urdf.xacro")
    models_path = os.path.join(description_share, "models")
    bridge_config = os.path.join(bringup_share, "config", "bridge.yaml")
    rviz_config = os.path.join(description_share, "rviz", "sim.rviz")
    default_waypoints = os.path.join(bringup_share, "config", "waypoints_gps.yaml")

    robot_description = Command([FindExecutable(name="xacro"), " ", xacro_file])
    resource_path = ":".join([
        models_path,
        os.path.dirname(description_share),
        os.path.dirname(bringup_share),
    ])
    cleanup_command = (
        "self=$$; "
        "pgrep -af 'gz sim server|gz sim -r|ros_gz_bridge/parameter_bridge|"
        "p3at_bringup/lib/p3at_bringup/clock_sanitizer|"
        "p3at_bringup/lib/p3at_bringup/odom_sanitizer|"
        "p3at_bringup/lib/p3at_bringup/joint_state_sanitizer|"
        "p3at_bringup/lib/p3at_bringup/fake_gps_node|"
        "robot_state_publisher/robot_state_publisher' "
        "| awk -v self=\"$self\" '$1 != self {print $1}' | xargs -r kill -TERM; "
        "sleep 1"
    )

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r ", world]}.items(),
        condition=UnlessCondition(headless),
    )

    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r -s ", world]}.items(),
        condition=IfCondition(headless),
    )

    cleanup = ExecuteProcess(
        cmd=["bash", "-lc", cleanup_command],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description},
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[
            {"config_file": bridge_config},
            {"use_sim_time": use_sim_time},
        ],
    )

    clock_sanitizer = Node(
        package="p3at_bringup",
        executable="clock_sanitizer",
        name="clock_sanitizer",
        output="screen",
    )

    odom_sanitizer = Node(
        package="p3at_bringup",
        executable="odom_sanitizer",
        name="odom_sanitizer",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": "/odom_raw"},
            {"output_topic": "/odom"},
            {"publish_filtered": False},
            {"publish_tf": False},
        ],
    )

    joint_state_sanitizer = Node(
        package="p3at_bringup",
        executable="joint_state_sanitizer",
        name="joint_state_sanitizer",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    fake_gps_node = Node(
        package="p3at_bringup",
        executable="fake_gps_node",
        name="fake_gps_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"waypoints_file": waypoints_file},
            {"noise_horizontal_std_m": gps_noise_h},
            {"noise_vertical_std_m": gps_noise_v},
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

    teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix="xterm -e",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(teleop),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Launch RViz.",
        ),
        DeclareLaunchArgument(
            "teleop",
            default_value="false",
            description="Launch teleop_twist_keyboard for manual driving.",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo server only (no GUI). Use RViz for visualization.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([bringup_share, "worlds", "james_oval.sdf"]),
            description="Path to the Gazebo world file.",
        ),
        DeclareLaunchArgument(
            "waypoints_file",
            default_value=default_waypoints,
            description="Part2 waypoints file used by fake GPS origin anchoring.",
        ),
        DeclareLaunchArgument(
            "gps_noise_h",
            default_value="1.5",
            description="Horizontal sigma (meters) for fake GPS noise.",
        ),
        DeclareLaunchArgument(
            "gps_noise_v",
            default_value="3.0",
            description="Vertical sigma (meters) for fake GPS noise.",
        ),
        SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=resource_path),
        cleanup,
        TimerAction(
            period=1.5,
            actions=[
                gazebo_gui,
                gazebo_headless,
                robot_state_publisher,
                bridge,
                clock_sanitizer,
                odom_sanitizer,
                joint_state_sanitizer,
                fake_gps_node,
            ],
        ),
        rviz_node,
        teleop_node,
    ])
