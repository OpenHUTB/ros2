#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_actuator_fault_sim 启动文件。

用法::

    ros2 launch uav_actuator_fault_sim fault_sim.launch.py
    ros2 launch uav_actuator_fault_sim fault_sim.launch.py \\
        scene:=corridor fault:=single_loss rotor:=2 onset:=3.0

本包是**单进程**的：MuJoCo 仿真与 ROS 节点跑在同一个 Python 进程里。
底座 uav_mujoco_sim 之所以拆成"仿真服务端 + 桥接"两个进程，是为了让
rclpy(3.8) 与 MuJoCo(需 3.10) 共存于 Ubuntu 20.04；而本作业的目标平台是
Ubuntu 22.04 + Humble，rclpy 本身就是 3.10，没有版本冲突，单进程更简单。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE_NAME = "uav_actuator_fault_sim"


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory(PACKAGE_NAME)
    default_params = os.path.join(share, "config", "faults.yaml")

    args = [
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="参数文件路径"),
        DeclareLaunchArgument("scene", default_value="open_ground",
                              description="物理场景"),
        DeclareLaunchArgument("fault", default_value="single_loss",
                              description="故障类型"),
        DeclareLaunchArgument("rotor", default_value="0",
                              description="故障桨序号 0~3"),
        DeclareLaunchArgument("onset", default_value="2.0",
                              description="故障注入时刻（秒）"),
        DeclareLaunchArgument("severity", default_value="0.0",
                              description="故障强度"),
        DeclareLaunchArgument("seconds", default_value="0.0",
                              description="运行时长，0 表示一直运行"),
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable="fault_node",
        name=PACKAGE_NAME,
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "scene": LaunchConfiguration("scene"),
                "fault": LaunchConfiguration("fault"),
                "rotor": ParameterValue(LaunchConfiguration("rotor"), value_type=int),
                "onset": ParameterValue(LaunchConfiguration("onset"), value_type=float),
                "severity": ParameterValue(LaunchConfiguration("severity"), value_type=float),
                "seconds": ParameterValue(LaunchConfiguration("seconds"), value_type=float),
            },
        ],
    )

    return LaunchDescription(args + [node])
