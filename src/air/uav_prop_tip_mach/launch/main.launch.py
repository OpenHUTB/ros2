#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动桨尖马赫数核算的离线入口（ExecuteProcess, 无需 rclpy）.

    ros2 launch uav_prop_tip_mach main.launch.py
    ros2 launch uav_prop_tip_mach main.launch.py mass:=1.2 diameter:=0.127 max_rpm:=16000
"""
import launch
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

PACKAGE = 'uav_prop_tip_mach'

ARGUMENTS = (
    ('mass', '2.0', '整机质量 kg'),
    ('diameter', '0.254', '单桨直径 m'),
    ('rotor_count', '4', '旋翼数量'),
    ('hover_rpm', '6500.0', '悬停转速 rpm'),
    ('max_rpm', '9000.0', '最大转速 rpm'),
    ('temperature', '15.0', '环境温度 C'),
    ('output', 'outputs', '输出目录'),
)


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument(name, default_value=default, description=text)
        for name, default, text in ARGUMENTS
    ]
    cmd = ['prop_tip_mach',
           '--mass', LaunchConfiguration('mass'),
           '--diameter', LaunchConfiguration('diameter'),
           '--rotor-count', LaunchConfiguration('rotor_count'),
           '--hover-rpm', LaunchConfiguration('hover_rpm'),
           '--max-rpm', LaunchConfiguration('max_rpm'),
           '--temperature', LaunchConfiguration('temperature'),
           '--output', LaunchConfiguration('output')]
    run = ExecuteProcess(cmd=cmd, output='screen')
    return launch.LaunchDescription(declarations + [run])
