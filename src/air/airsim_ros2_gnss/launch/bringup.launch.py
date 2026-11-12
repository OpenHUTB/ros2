"""Launch the AirSim ROS 2 GNSS bridge."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_file = (
        Path(get_package_share_directory("airsim_ros2_gnss"))
        / "config"
        / "gnss.yaml"
    )
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    vehicle_name = LaunchConfiguration("vehicle_name")
    gps_name = LaunchConfiguration("gps_name")
    fix_topic = LaunchConfiguration("fix_topic")
    frame_id = LaunchConfiguration("frame_id")
    publish_rate = LaunchConfiguration("publish_rate")
    connection_timeout = LaunchConfiguration("connection_timeout")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("port", default_value="41451"),
            DeclareLaunchArgument("vehicle_name", default_value=""),
            DeclareLaunchArgument("gps_name", default_value=""),
            DeclareLaunchArgument("fix_topic", default_value="/uav/gps/fix"),
            DeclareLaunchArgument("frame_id", default_value="gps_link"),
            DeclareLaunchArgument("publish_rate", default_value="5.0"),
            DeclareLaunchArgument(
                "connection_timeout",
                default_value="1.0",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="airsim_ros2_gnss",
                executable="main",
                name="airsim_gnss_bridge",
                output="screen",
                parameters=[
                    str(config_file),
                    {
                        "host": host,
                        "port": port,
                        "vehicle_name": vehicle_name,
                        "gps_name": gps_name,
                        "fix_topic": fix_topic,
                        "frame_id": frame_id,
                        "publish_rate": publish_rate,
                        "connection_timeout": connection_timeout,
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
        ]
    )
