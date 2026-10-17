"""ROS 2 launch entry point for the AEB node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    params = os.path.join(get_package_share_directory("aeb_collision_avoidance"), "config", "aeb_params.yaml")
    return LaunchDescription(
        [
            Node(
                package="aeb_collision_avoidance",
                executable="aeb_node",
                name="aeb_collision_avoidance",
                output="screen",
                parameters=[ParameterFile(params, allow_substs=True)],
            )
        ]
    )
