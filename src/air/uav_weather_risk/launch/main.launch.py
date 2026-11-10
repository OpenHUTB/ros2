import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("uav_weather_risk"), "config", "demo.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("duration", default_value="0.0"),
            DeclareLaunchArgument("seed", default_value="42"),
            DeclareLaunchArgument("samples", default_value="64"),
            DeclareLaunchArgument("assessment_period", default_value="0.5"),
            Node(
                package="uav_weather_risk",
                executable="weather_demo",
                name="uav_weather_demo",
                output="screen",
                parameters=[
                    config,
                    {
                        "duration_s": ParameterValue(
                            LaunchConfiguration("duration"), value_type=float
                        ),
                        "seed": ParameterValue(LaunchConfiguration("seed"), value_type=int),
                    },
                ],
            ),
            Node(
                package="uav_weather_risk",
                executable="weather_risk",
                name="uav_weather_risk",
                output="screen",
                parameters=[
                    config,
                    {
                        "samples": ParameterValue(
                            LaunchConfiguration("samples"), value_type=int
                        ),
                        "assessment_period_s": ParameterValue(
                            LaunchConfiguration("assessment_period"), value_type=float
                        ),
                        "seed": ParameterValue(LaunchConfiguration("seed"), value_type=int),
                    },
                ],
            ),
        ]
    )
