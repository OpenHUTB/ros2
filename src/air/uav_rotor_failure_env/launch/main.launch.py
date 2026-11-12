# -*- coding: utf-8 -*-
"""启动失效包线诊断节点。

    ros2 launch uav_rotor_failure_env main.launch.py
    ros2 launch uav_rotor_failure_env main.launch.py config:=octo failed_rotor:=3

    ros2 topic echo /uav/allocation_report
    ros2 topic echo /uav/allocation_margin
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("uav_rotor_failure_env")
    params = os.path.join(share, "config", "params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value="quad",
                              description="quad / hexa / octo"),
        DeclareLaunchArgument("failed_rotor", default_value="0",
                              description="失效桨编号"),
        DeclareLaunchArgument("mass", default_value="2.0", description="机重 kg"),
        Node(
            package="uav_rotor_failure_env",
            executable="allocation_node",
            name="rotor_failure_env_node",
            output="screen",
            parameters=[params, {
                "config": LaunchConfiguration("config"),
                "failed_rotor": LaunchConfiguration("failed_rotor"),
                "mass": LaunchConfiguration("mass"),
            }],
        ),
    ])
