"""Launch the AirSim ROS 2 barometer bridge."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    config_file = (
        Path(get_package_share_directory("airsim_ros2_barometer"))
        / "config"
        / "barometer.yaml"
    )
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    vehicle_name = LaunchConfiguration("vehicle_name")
    barometer_name = LaunchConfiguration("barometer_name")
    pressure_topic = LaunchConfiguration("pressure_topic")
    altitude_topic = LaunchConfiguration("altitude_topic")
    qnh_topic = LaunchConfiguration("qnh_topic")
    frame_id = LaunchConfiguration("frame_id")
    publish_rate = LaunchConfiguration("publish_rate")
    rpc_timeout = LaunchConfiguration("rpc_timeout")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("port", default_value="41451"),
            DeclareLaunchArgument("vehicle_name", default_value=""),
            DeclareLaunchArgument("barometer_name", default_value=""),
            DeclareLaunchArgument(
                "pressure_topic",
                default_value="/uav/barometer/pressure",
            ),
            DeclareLaunchArgument(
                "altitude_topic",
                default_value="/uav/barometer/altitude",
            ),
            DeclareLaunchArgument(
                "qnh_topic",
                default_value="/uav/barometer/qnh",
            ),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("publish_rate", default_value="20.0"),
            DeclareLaunchArgument(
                "rpc_timeout",
                default_value="1.0",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="airsim_ros2_barometer",
                executable="main",
                name="airsim_barometer_bridge",
                output="screen",
                parameters=[
                    str(config_file),
                    {
                        "host": ParameterValue(host, value_type=str),
                        "port": ParameterValue(port, value_type=int),
                        "vehicle_name": ParameterValue(
                            vehicle_name,
                            value_type=str,
                        ),
                        "barometer_name": ParameterValue(
                            barometer_name,
                            value_type=str,
                        ),
                        "pressure_topic": ParameterValue(
                            pressure_topic,
                            value_type=str,
                        ),
                        "altitude_topic": ParameterValue(
                            altitude_topic,
                            value_type=str,
                        ),
                        "qnh_topic": ParameterValue(
                            qnh_topic,
                            value_type=str,
                        ),
                        "frame_id": ParameterValue(frame_id, value_type=str),
                        "publish_rate": ParameterValue(
                            publish_rate,
                            value_type=float,
                        ),
                        "rpc_timeout": ParameterValue(
                            rpc_timeout,
                            value_type=float,
                        ),
                        "use_sim_time": ParameterValue(
                            use_sim_time,
                            value_type=bool,
                        ),
                    },
                ],
            ),
        ]
    )
