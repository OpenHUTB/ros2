# -*- coding: utf-8 -*-
"""ROS 2 launch：发布 ZMP 规划的台阶行走轨迹。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("direction", default_value="up"),
        DeclareLaunchArgument("step_height", default_value="0.15"),
        DeclareLaunchArgument("step_depth", default_value="0.30"),
    ]
    node = Node(package="humanoid_stair_zmp_preview", executable="stair_zmp_node",
                name="stair_zmp_node", output="screen",
                parameters=[{"direction": LaunchConfiguration("direction"),
                             "step_height": LaunchConfiguration("step_height"),
                             "step_depth": LaunchConfiguration("step_depth")}])
    return LaunchDescription(args + [node])