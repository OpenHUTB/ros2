#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键启动四旋翼串级 PID 仿真实验。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = "quad_cascade_pid_hover_wind"
    share = get_package_share_directory(pkg)
    default_params = os.path.join(share, "config", "params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("output_dir", default_value="/tmp/quad_results"),
        DeclareLaunchArgument("duration", default_value="40.0"),
        DeclareLaunchArgument("sim_dt", default_value="0.002"),
        DeclareLaunchArgument("ctrl_rate", default_value="200.0"),
        DeclareLaunchArgument("log_rate", default_value="50.0"),

        Node(
            package=pkg, executable="sim_node", name="quad_sim",
            output="screen",
            parameters=[
                {"params_file": LaunchConfiguration("params_file")},
                {"dt": LaunchConfiguration("sim_dt")},
            ],
        ),
        Node(
            package=pkg, executable="controller_node", name="quad_controller",
            output="screen",
            parameters=[
                {"params_file": LaunchConfiguration("params_file")},
                {"rate": LaunchConfiguration("ctrl_rate")},
            ],
        ),
        Node(
            package=pkg, executable="logger_node", name="quad_logger",
            output="screen",
            parameters=[
                {"output_dir": LaunchConfiguration("output_dir")},
                {"log_rate": LaunchConfiguration("log_rate")},
                {"duration": LaunchConfiguration("duration")},
            ],
        ),
    ])
