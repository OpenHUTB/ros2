import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("vio_time_offset_estimation"), "config", "demo.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("offset", default_value="0.025"),
            Node(
                package="vio_time_offset_estimation",
                executable="vio_sim",
                name="vio_sim",
                output="screen",
                parameters=[
                    config,
                    {
                        "offset_s": ParameterValue(
                            LaunchConfiguration("offset"), value_type=float
                        )
                    },
                ],
            ),
            Node(
                package="vio_time_offset_estimation",
                executable="time_offset_estimator",
                name="time_offset_estimator",
                output="screen",
                parameters=[config],
            ),
        ]
    )
