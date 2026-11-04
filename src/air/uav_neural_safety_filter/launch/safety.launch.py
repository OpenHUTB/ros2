#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_neural_safety_filter 启动入口。

    ros2 launch uav_neural_safety_filter safety.launch.py
    ros2 launch uav_neural_safety_filter safety.launch.py with_sim:=true
    ros2 launch uav_neural_safety_filter safety.launch.py model:=/path/to/safety_policy.npz

``with_sim:=true`` 时一并拉起底座（仿真服务端 + 桥接），否则只起安全层，
假定 /lidar/points、/uav/odom 与 /uav/cmd_vel_nominal 已在别处运行。
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    base_share = get_package_share_directory("uav_mujoco_sim")
    cfg = os.path.join(get_package_share_directory("uav_neural_safety_filter"),
                       "config", "safety.yaml")

    args = [
        DeclareLaunchArgument("with_sim", default_value="false"),
        DeclareLaunchArgument("scene", default_value="obstacle_field"),
        DeclareLaunchArgument("quality", default_value="fast"),
        DeclareLaunchArgument("model", default_value=""),
        DeclareLaunchArgument("rate_hz", default_value="20.0"),
        DeclareLaunchArgument("nominal", default_value="true",
                              description="同时起一个朝目标飞的名义控制器"),
        DeclareLaunchArgument("goal_x", default_value="8.0"),
        DeclareLaunchArgument("goal_y", default_value="8.0"),
        DeclareLaunchArgument("goal_z", default_value="1.6"),
        DeclareLaunchArgument("speed", default_value="2.5"),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_share, "launch", "main.launch.py")),
        launch_arguments={"scene": LaunchConfiguration("scene"),
                          "quality": LaunchConfiguration("quality"),
                          "keyboard": "false"}.items(),
        condition=IfCondition(LaunchConfiguration("with_sim")))

    safety = Node(
        package="uav_neural_safety_filter", executable="safety_node",
        name="uav_safety_filter", output="screen",
        parameters=[cfg, {"model": LaunchConfiguration("model"),
                          "rate_hz": LaunchConfiguration("rate_hz")}])

    nominal = Node(
        package="uav_neural_safety_filter", executable="nominal_node",
        name="uav_nominal_controller", output="screen",
        parameters=[{"goal_x": LaunchConfiguration("goal_x"),
                     "goal_y": LaunchConfiguration("goal_y"),
                     "goal_z": LaunchConfiguration("goal_z"),
                     "speed": LaunchConfiguration("speed")}],
        condition=IfCondition(LaunchConfiguration("nominal")))

    return LaunchDescription(args + [sim,
                                     TimerAction(period=3.0,
                                                 actions=[safety, nominal])])
