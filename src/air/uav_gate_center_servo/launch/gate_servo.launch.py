#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动单门视觉伺服节点；with_sim:=true 时同时拉起 MuJoCo 底座。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    base_share = get_package_share_directory("uav_mujoco_sim")
    cfg = os.path.join(get_package_share_directory("uav_gate_center_servo"),
                       "config", "gate_servo.yaml")

    args = [
        DeclareLaunchArgument("with_sim", default_value="false",
                              description="true 则一并拉起底座仿真与桥接"),
        DeclareLaunchArgument("scene", default_value="gate_track"),
        DeclareLaunchArgument("quality", default_value="fast"),
        DeclareLaunchArgument("model", default_value=""),
        DeclareLaunchArgument("rate_hz", default_value="20.0"),
        DeclareLaunchArgument("publish_view", default_value="true"),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_share, "launch", "main.launch.py")),
        # 不能传 headless:=true：底座会关闭视觉，/camera/depth 变成全零。
        launch_arguments={"scene": LaunchConfiguration("scene"),
                          "quality": LaunchConfiguration("quality"),
                          "keyboard": "false"}.items(),
        condition=IfCondition(LaunchConfiguration("with_sim")))

    node = Node(
        package="uav_gate_center_servo", executable="gate_servo_node",
        name="gate_servo_node", output="screen",
        parameters=[cfg, {
            "model": LaunchConfiguration("model"),
            "rate_hz": LaunchConfiguration("rate_hz"),
            "publish_view": LaunchConfiguration("publish_view"),
        }])

    return LaunchDescription(args + [sim, TimerAction(period=3.0,
                                                       actions=[node])])
