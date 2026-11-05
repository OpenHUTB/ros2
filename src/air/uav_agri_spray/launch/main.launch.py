#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch the online spray progress node."""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("uav_agri_spray"),
                       "config", "uav_agri_spray.yaml")
    return LaunchDescription([
        Node(
            package="uav_agri_spray",
            executable="spray",
            name="spray_node",
            output="screen",
            parameters=[cfg],
        ),
    ])
