#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_neural_vio_slam 启动入口。

    ros2 launch uav_neural_vio_slam vio.launch.py
    ros2 launch uav_neural_vio_slam vio.launch.py with_sim:=true scene:=urban_block
    ros2 launch uav_neural_vio_slam vio.launch.py backend:=fastslam

with_sim:=true 时一并拉起底座（仿真服务端 + 桥接），否则只起 VIO 节点，
假定 /camera/depth 与仿真端已在别处运行。
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
    cfg = os.path.join(get_package_share_directory("uav_neural_vio_slam"),
                       "config", "vio.yaml")

    args = [
        DeclareLaunchArgument("backend", default_value="ekf",
                              choices=["odom", "ekf", "ekf_learned", "fastslam"],
                              description="SLAM 后端"),
        DeclareLaunchArgument("model_dir", default_value="",
                              description="预训练权重目录（留空用包内 data/）"),
        DeclareLaunchArgument("sim_host", default_value="127.0.0.1",
                              description="仿真端地址"),
        DeclareLaunchArgument("gps_period", default_value="1.0"),
        DeclareLaunchArgument("gps_sigma", default_value="0.5"),
        DeclareLaunchArgument("outage", default_value="0.0",
                              description="GPS 中断时长（秒），0 表示不中断"),
        DeclareLaunchArgument("with_sim", default_value="false",
                              description="true 则一并拉起底座仿真与桥接"),
        DeclareLaunchArgument("scene", default_value="urban_block"),
        DeclareLaunchArgument("quality", default_value="fast"),
        DeclareLaunchArgument("capture", default_value="false",
                              description="true 则录制 rosbag（需 ros2 bag）"),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_share, "launch", "main.launch.py")),
        # 不能传 headless:=true：底座那个开关会直接把渲染器置空，/camera/rgb 与
        # /camera/depth 全零，VIO 前端学到的就是空图。无显示环境由底座切 osmesa。
        launch_arguments={"scene": LaunchConfiguration("scene"),
                          "quality": LaunchConfiguration("quality"),
                          "keyboard": "false"}.items(),
        condition=IfCondition(LaunchConfiguration("with_sim")))

    vio = Node(
        package="uav_neural_vio_slam", executable="vio_node",
        name="uav_vio_slam", output="screen",
        parameters=[cfg, {
            "vio/backend": LaunchConfiguration("backend"),
            "vio/model_dir": LaunchConfiguration("model_dir"),
            "vio/gps_period": LaunchConfiguration("gps_period"),
            "vio/gps_sigma": LaunchConfiguration("gps_sigma"),
            "sim/host": LaunchConfiguration("sim_host"),
        }])

    # 桥接自带 30 s 重试，这里只错峰
    return LaunchDescription(args + [sim, TimerAction(period=3.0, actions=[vio])])
