import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _maybe_launch_driver(context, package_share):
    launch_driver = LaunchConfiguration("launch_camera_driver").perform(context).lower() == "true"
    if not launch_driver:
        return []

    driver_share = get_package_share_directory("depthai_ros_driver_v3")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(driver_share, "launch", "driver.launch.py")),
            launch_arguments={
                "rs_compat": "true",
                "use_rviz": "false",
                "params_file": os.path.join(package_share, "config", "luxonis_rgbd_rs.yaml"),
            }.items(),
        )
    ]


def generate_launch_description():
    package_share = get_package_share_directory("p3at_vision")

    run_id = LaunchConfiguration("run_id")
    rgb_topic = LaunchConfiguration("rgb_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    navigation_topic = LaunchConfiguration("navigation_topic")
    vision_topic = LaunchConfiguration("vision_topic")
    use_depth = LaunchConfiguration("use_depth")
    save_dir = LaunchConfiguration("save_dir")
    save_raw = LaunchConfiguration("save_raw")
    save_annotated = LaunchConfiguration("save_annotated")

    vision_node = Node(
        package="p3at_vision",
        executable="vision_node",
        name="vision_node",
        output="screen",
        parameters=[
            {
                "run_id": run_id,
                "rgb_topic": rgb_topic,
                "depth_topic": depth_topic,
                "camera_info_topic": camera_info_topic,
                "navigation_topic": navigation_topic,
                "vision_topic": vision_topic,
                "use_depth": use_depth,
                "save_dir": save_dir,
                "save_raw": save_raw,
                "save_annotated": save_annotated,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("run_id", default_value="vision_dev"),
            DeclareLaunchArgument("rgb_topic", default_value="/camera/camera/color/image_raw"),
            DeclareLaunchArgument("depth_topic", default_value="/camera/camera/depth/image_rect_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info"),
            DeclareLaunchArgument("navigation_topic", default_value="/mission/navigation"),
            DeclareLaunchArgument("vision_topic", default_value="/mission/vision"),
            DeclareLaunchArgument("use_depth", default_value="true"),
            DeclareLaunchArgument("launch_camera_driver", default_value="true"),
            DeclareLaunchArgument(
                "save_dir",
                default_value=str(Path.cwd() / "artifacts" / "part2_runs" / "vision_dev" / "photos"),
            ),
            DeclareLaunchArgument("save_raw", default_value="true"),
            DeclareLaunchArgument("save_annotated", default_value="true"),
            OpaqueFunction(function=lambda context: _maybe_launch_driver(context, package_share)),
            vision_node,
        ]
    )
