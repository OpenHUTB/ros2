"""ROS 2 launch entry point for the pix2cmd node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    params = os.path.join(get_package_share_directory("pix2cmd_end2end_driving"), "config", "params.yaml")
    return LaunchDescription([
        Node(
            package="pix2cmd_end2end_driving",
            executable="pix2cmd_node",
            name="pix2cmd_end2end_driving",
            output="screen",
            parameters=[ParameterFile(params, allow_substs=True)],
        )
    ])