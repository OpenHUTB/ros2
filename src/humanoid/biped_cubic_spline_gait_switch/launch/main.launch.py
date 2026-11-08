#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动双足多步态平滑切换仿真（控制 + 仿真 + 记录）。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("biped_cubic_spline_gait_switch")
    default_config = os.path.join(pkg_share, "config", "params.yaml")

    mode = LaunchConfiguration("mode")
    output_dir = LaunchConfiguration("output_dir")
    duration = LaunchConfiguration("duration")
    config = LaunchConfiguration("config")

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="spline",
                              description="步态切换方式：spline 或 direct"),
        DeclareLaunchArgument("output_dir", default_value="results/run_spline"),
        DeclareLaunchArgument("duration", default_value="34.0"),
        DeclareLaunchArgument("config", default_value=default_config),
        Node(
            package="biped_cubic_spline_gait_switch",
            executable="controller_node",
            name="biped_controller",
            output="screen",
            parameters=[{
                "config": config,
                "mode": mode,
                "transition_time": 0.6,
                "control_rate": 200.0,
            }],
        ),
        Node(
            package="biped_cubic_spline_gait_switch",
            executable="sim_node",
            name="biped_sim",
            output="screen",
            parameters=[{"config": config}],
        ),
        Node(
            package="biped_cubic_spline_gait_switch",
            executable="logger_node",
            name="biped_logger",
            output="screen",
            parameters=[{
                "output_dir": output_dir,
                "duration": duration,
                "mode": mode,
            }],
        ),
    ])
