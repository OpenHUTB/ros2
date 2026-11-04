#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_depth_gate_racing_e2e 启动入口。

    ros2 launch uav_depth_gate_racing_e2e racing.launch.py
    ros2 launch uav_depth_gate_racing_e2e racing.launch.py with_sim:=true
    ros2 launch uav_depth_gate_racing_e2e racing.launch.py model:=/path/to/policy_ppo.npz

``with_sim:=true`` 时一并拉起底座（仿真服务端 + 桥接），否则只起竞速节点，
假定 /camera/depth 与 /uav/odom 已在别处运行。
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
    cfg = os.path.join(get_package_share_directory("uav_depth_gate_racing_e2e"),
                       "config", "racing.yaml")

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
        # 不能传 headless:=true：底座那个开关会直接把渲染器置空，/camera/depth
        # 变成全零图，策略等于闭着眼飞。无显示环境由底座自动切 osmesa 解决。
        launch_arguments={"scene": LaunchConfiguration("scene"),
                          "quality": LaunchConfiguration("quality"),
                          "keyboard": "false"}.items(),
        condition=IfCondition(LaunchConfiguration("with_sim")))

    node = Node(
        package="uav_depth_gate_racing_e2e", executable="racing_node",
        name="uav_gate_racing", output="screen",
        parameters=[cfg, {
            "model": LaunchConfiguration("model"),
            "rate_hz": LaunchConfiguration("rate_hz"),
            "publish_view": LaunchConfiguration("publish_view"),
        }])

    return LaunchDescription(args + [sim, TimerAction(period=3.0, actions=[node])])
