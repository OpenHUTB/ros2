import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("mag_hard_soft_iron_calibration"), "config", "demo.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("samples", default_value="600"),
            Node(
                package="mag_hard_soft_iron_calibration",
                executable="mag_sim",
                name="mag_sim",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="mag_hard_soft_iron_calibration",
                executable="mag_calibrator",
                name="mag_calibrator",
                output="screen",
                parameters=[
                    config,
                    {
                        "samples": ParameterValue(
                            LaunchConfiguration("samples"), value_type=int
                        )
                    },
                ],
            ),
        ]
    )
