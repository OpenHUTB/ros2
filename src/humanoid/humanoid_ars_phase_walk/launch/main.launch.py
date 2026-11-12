# -*- coding: utf-8 -*-
"""ROS 2 launch：发布 CPG 基线的关节目标。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("period", default_value="0.02"),
    ]
    return LaunchDescription(args + [
        Node(package="humanoid_ars_phase_walk", executable="ars_walk_node",
             name="ars_walk_node", output="screen",
             parameters=[{"period": LaunchConfiguration("period")}])
    ])