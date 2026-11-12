"""Launch the AirSim ROS 2 IMU bridge."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_file = (
        Path(get_package_share_directory("airsim_ros2_imu"))
        / "config"
        / "imu.yaml"
    )
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    vehicle_name = LaunchConfiguration("vehicle_name")
    imu_name = LaunchConfiguration("imu_name")
    imu_topic = LaunchConfiguration("imu_topic")
    frame_id = LaunchConfiguration("frame_id")
    publish_rate = LaunchConfiguration("publish_rate")
    stale_timeout = LaunchConfiguration("stale_timeout")
    use_source_timestamp = LaunchConfiguration("use_source_timestamp")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("port", default_value="41451"),
            DeclareLaunchArgument("vehicle_name", default_value=""),
            DeclareLaunchArgument("imu_name", default_value=""),
            DeclareLaunchArgument("imu_topic", default_value="/uav/imu"),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("publish_rate", default_value="50.0"),
            DeclareLaunchArgument("stale_timeout", default_value="0.25"),
            DeclareLaunchArgument(
                "use_source_timestamp",
                default_value="true",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="airsim_ros2_imu",
                executable="main",
                name="airsim_imu_bridge",
                output="screen",
                parameters=[
                    str(config_file),
                    {
                        "host": host,
                        "port": port,
                        "vehicle_name": vehicle_name,
                        "imu_name": imu_name,
                        "imu_topic": imu_topic,
                        "frame_id": frame_id,
                        "publish_rate": publish_rate,
                        "stale_timeout": stale_timeout,
                        "use_source_timestamp": use_source_timestamp,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
        ]
    )
