# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("mujoco_musculoskeletal_knee_tracking")
    params_file = os.path.join(pkg, "config", "params.yaml")

    return LaunchDescription([
        Node(
            package="mujoco_musculoskeletal_knee_tracking",
            executable="mujoco_musculoskeletal_knee_tracking_node",
            name="mujoco_musculoskeletal_knee_tracking_node",
            output="screen",
            parameters=[params_file],
        )
    ])
