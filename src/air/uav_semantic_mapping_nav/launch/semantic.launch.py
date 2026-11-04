#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_semantic_mapping_nav 启动入口。

    ros2 launch uav_semantic_mapping_nav semantic.launch.py
    ros2 launch uav_semantic_mapping_nav semantic.launch.py with_sim:=true
    ros2 launch uav_semantic_mapping_nav semantic.launch.py use_truth:=true

``with_sim:=true`` 时一并拉起底座（仿真服务端 + 桥接），否则只起语义节点，
假定 /camera/* 与 /uav/odom 已在别处运行。
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
    cfg = os.path.join(get_package_share_directory("uav_semantic_mapping_nav"),
                       "config", "semantic.yaml")

    args = [
        DeclareLaunchArgument("with_sim", default_value="false",
                              description="true 则一并拉起底座仿真与桥接"),
        DeclareLaunchArgument("scene", default_value="urban_block"),
        DeclareLaunchArgument("quality", default_value="fast"),
        DeclareLaunchArgument("use_truth", default_value="false",
                              description="true 则用真值分割替代网络预测"),
        DeclareLaunchArgument("model_dir", default_value=""),
        DeclareLaunchArgument("map_res", default_value="0.20"),
        DeclareLaunchArgument("fovy", default_value="60.0"),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_share, "launch", "main.launch.py")),
        # 不能传 headless:=true：底座那个开关会直接把渲染器置空，/camera/rgb 与
        # /camera/depth 全零，分割网络无从下手。无显示环境由底座自动切 osmesa。
        launch_arguments={"scene": LaunchConfiguration("scene"),
                          "quality": LaunchConfiguration("quality"),
                          "keyboard": "false"}.items(),
        condition=IfCondition(LaunchConfiguration("with_sim")))

    node = Node(
        package="uav_semantic_mapping_nav", executable="semantic_node",
        name="uav_semantic_nav", output="screen",
        parameters=[cfg, {
            "map.use_truth_seg": LaunchConfiguration("use_truth"),
            "model_dir": LaunchConfiguration("model_dir"),
            "map.res": LaunchConfiguration("map_res"),
            "cam.fovy_deg": LaunchConfiguration("fovy"),
        }])

    return LaunchDescription(args + [sim, TimerAction(period=3.0, actions=[node])])
