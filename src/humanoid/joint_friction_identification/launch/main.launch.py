import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("joint_friction_identification"), "config", "demo.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="joint_friction_identification",
                executable="friction_sim",
                name="friction_sim",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="joint_friction_identification",
                executable="friction_identifier",
                name="friction_identifier",
                output="screen",
                parameters=[config],
            ),
        ]
    )
