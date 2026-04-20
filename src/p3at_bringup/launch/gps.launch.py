from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gps_port = LaunchConfiguration("gps_port")
    gps_baud = LaunchConfiguration("gps_baud")
    gps_frame_id = LaunchConfiguration("gps_frame_id")

    gps_node = Node(
        package="nmea_navsat_driver",
        executable="nmea_serial_driver",
        name="gps_nmea_driver",
        output="screen",
        parameters=[
            {
                "port": gps_port,
                "baud": gps_baud,
                "frame_id": gps_frame_id,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gps_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("gps_baud", default_value="9600"),
            DeclareLaunchArgument("gps_frame_id", default_value="gps_link"),
            gps_node,
        ]
    )
