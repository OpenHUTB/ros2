import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("rtk_ambiguity_resolution"), "config", "demo.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("windows", default_value="0"),
            Node(
                package="rtk_ambiguity_resolution",
                executable="rtk_sim",
                name="rtk_sim",
                output="screen",
                on_exit=Shutdown(),
                parameters=[
                    config,
                    {
                        "windows": ParameterValue(
                            LaunchConfiguration("windows"), value_type=int
                        )
                    },
                ],
            ),
            Node(
                package="rtk_ambiguity_resolution",
                executable="rtk_solver",
                name="rtk_solver",
                output="screen",
                parameters=[config],
            ),
        ]
    )
