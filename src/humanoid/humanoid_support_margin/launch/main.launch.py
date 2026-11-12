#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动核算的离线入口（ExecuteProcess, 无需 rclpy）.

    ros2 launch humanoid_support_margin main.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

PACKAGE = 'humanoid_support_margin'

ARGUMENTS = (
    ('mass', '40.0', '质量 kg'),
    ('foot1-x', '-0.10', '左脚中心 x m'),
    ('foot2-x', '0.10', '右脚中心 x m'),
    ('foot-width', '0.10', '脚掌宽 m'),
    ('foot-length', '0.25', '脚掌长 m'),
    ('com-x', '0.01', '重心投影 x m'),
    ('com-y', '0.0', '重心投影 y m'),
    ('com-height', '0.8', '重心高度 m'),
    ('output', 'outputs', '输出目录'),
)


def _launch_setup(context):
    """按实际参数拼出命令行."""
    cmd = ['humanoid_support_margin',
           '--mass', LaunchConfiguration('mass').perform(context),
           '--foot1-x', LaunchConfiguration('foot1-x').perform(context),
           '--foot2-x', LaunchConfiguration('foot2-x').perform(context),
           '--foot-width', LaunchConfiguration('foot-width').perform(context),
           '--foot-length', LaunchConfiguration('foot-length').perform(context),
           '--com-x', LaunchConfiguration('com-x').perform(context),
           '--com-y', LaunchConfiguration('com-y').perform(context),
           '--com-height', LaunchConfiguration('com-height').perform(context),
           '--output', LaunchConfiguration('output').perform(context)]
    return [ExecuteProcess(cmd=cmd, output='screen')]


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument(flag, default_value=default, description=text)
        for flag, default, text in ARGUMENTS
    ]
    return LaunchDescription(declarations + [OpaqueFunction(function=_launch_setup)])
