import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("rov_thruster_allocation"), "config", "demo.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("scale", default_value="1.0"),
            Node(
                package="rov_thruster_allocation",
                executable="thruster_sim",
                name="thruster_sim",
                output="screen",
                parameters=[
                    config,
                    {
                        "scale": ParameterValue(
                            LaunchConfiguration("scale"), value_type=float
                        )
                    },
                ],
            ),
            Node(
                package="rov_thruster_allocation",
                executable="thruster_allocator",
                name="thruster_allocator",
                output="screen",
                parameters=[config],
            ),
        ]
    )
