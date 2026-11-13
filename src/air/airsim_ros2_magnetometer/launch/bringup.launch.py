"""Launch the AirSim ROS 2 magnetometer bridge."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    config_file = (
        Path(get_package_share_directory("airsim_ros2_magnetometer"))
        / "config"
        / "magnetometer.yaml"
    )
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    vehicle_name = LaunchConfiguration("vehicle_name")
    magnetometer_name = LaunchConfiguration("magnetometer_name")
    topic = LaunchConfiguration("magnetometer_topic")
    frame_id = LaunchConfiguration("frame_id")
    publish_rate = LaunchConfiguration("publish_rate")
    reconnect_period = LaunchConfiguration("reconnect_period")
    rpc_timeout = LaunchConfiguration("rpc_timeout")
    field_scale = LaunchConfiguration("field_scale")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("port", default_value="41451"),
            DeclareLaunchArgument("vehicle_name", default_value=""),
            DeclareLaunchArgument("magnetometer_name", default_value=""),
            DeclareLaunchArgument("magnetometer_topic", default_value="/uav/magnetometer"),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("publish_rate", default_value="20.0"),
            DeclareLaunchArgument("reconnect_period", default_value="2.0"),
            DeclareLaunchArgument("rpc_timeout", default_value="1.0"),
            DeclareLaunchArgument("field_scale", default_value="1.0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="airsim_ros2_magnetometer",
                executable="main",
                name="airsim_magnetometer_bridge",
                output="screen",
                parameters=[
                    str(config_file),
                    {
                        "host": host,
                        "port": ParameterValue(port, value_type=int),
                        "vehicle_name": ParameterValue(vehicle_name, value_type=str),
                        "magnetometer_name": ParameterValue(
                            magnetometer_name, value_type=str
                        ),
                        "magnetometer_topic": ParameterValue(topic, value_type=str),
                        "frame_id": ParameterValue(frame_id, value_type=str),
                        "publish_rate": ParameterValue(publish_rate, value_type=float),
                        "reconnect_period": ParameterValue(
                            reconnect_period, value_type=float
                        ),
                        "rpc_timeout": ParameterValue(rpc_timeout, value_type=float),
                        "field_scale": ParameterValue(field_scale, value_type=float),
                        "use_sim_time": ParameterValue(
                            use_sim_time,
                            value_type=bool,
                        ),
                    },
                ],
            ),
        ]
    )
