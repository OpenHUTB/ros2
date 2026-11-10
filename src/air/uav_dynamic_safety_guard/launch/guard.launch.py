#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动文件：仿真 + 标称控制器 + 安全层。

    # 全套（MuJoCo 仿真 + 桥接 + 安全层 + 演示标称控制器）
    ros2 launch uav_dynamic_safety_guard guard.launch.py with_sim:=true

    # 只起安全层（假定 /lidar/points、/uav/odom、/uav/cmd_vel_nominal 已在跑）
    ros2 launch uav_dynamic_safety_guard guard.launch.py with_sim:=false nominal:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("uav_dynamic_safety_guard")
    params = os.path.join(share, "config", "guard.yaml")

    with_sim = LaunchConfiguration("with_sim")
    nominal = LaunchConfiguration("nominal")
    model = LaunchConfiguration("model")

    args = [
        DeclareLaunchArgument("with_sim", default_value="true",
                              description="是否同时启动 uav_mujoco_sim 与桥接"),
        DeclareLaunchArgument("nominal", default_value="true",
                              description="是否启动演示用标称控制器"),
        DeclareLaunchArgument("model", default_value="",
                              description="策略权重路径；空则用包内 data/"),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("uav_mujoco_sim"), "launch",
            "main.launch.py")),
        condition=IfCondition(with_sim),
    )

    guard = Node(
        package="uav_dynamic_safety_guard",
        executable="guard_node",
        name="uav_dynamic_safety_guard",
        output="screen",
        parameters=[params, {"model": model}],
    )

    nominal_node = Node(
        package="uav_dynamic_safety_guard",
        executable="nominal_node",
        name="uav_nominal_controller",
        output="screen",
        parameters=[params],
        condition=IfCondition(nominal),
    )

    return LaunchDescription(args + [sim, guard, nominal_node])
