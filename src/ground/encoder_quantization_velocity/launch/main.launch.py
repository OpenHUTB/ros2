import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("encoder_quantization_velocity"), "config", "demo.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("cpr", default_value="4096"),
            DeclareLaunchArgument("duration_s", default_value="120.0"),
            Node(
                package="encoder_quantization_velocity",
                executable="encoder_sim",
                name="encoder_sim",
                output="screen",
                parameters=[
                    config,
                    {
                        "counts_per_revolution": ParameterValue(
                            LaunchConfiguration("cpr"), value_type=int
                        ),
                        "duration_s": ParameterValue(
                            LaunchConfiguration("duration_s"), value_type=float
                        ),
                    },
                ],
            ),
            Node(
                package="encoder_quantization_velocity",
                executable="velocity_estimator",
                name="velocity_estimator",
                output="screen",
                parameters=[config],
            ),
        ]
    )
