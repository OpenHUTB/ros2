#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动核算的离线入口（ExecuteProcess, 无需 rclpy）.

    ros2 launch uav_cg_trim_envelope main.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

PACKAGE = 'uav_cg_trim_envelope'

ARGUMENTS = (
    ('mass', '1.2', '起飞质量 kg'),
    ('arm-x', '0.25', '左右力臂 m'),
    ('arm-y', '0.25', '前后力臂 m'),
    ('dx', '3.0', '纵向重心偏移 mm'),
    ('dy', '2.0', '横向重心偏移 mm'),
    ('cog-depth', '30.0', '桨平面到重心高度 mm'),
    ('thrust-max', '6.0', '单电机最大拉力 N'),
    ('output', 'outputs', '输出目录'),
)


def _launch_setup(context):
    """按实际参数拼出命令行."""
    cmd = ['uav_cg_trim_envelope',
           '--mass', LaunchConfiguration('mass').perform(context),
           '--arm-x', LaunchConfiguration('arm-x').perform(context),
           '--arm-y', LaunchConfiguration('arm-y').perform(context),
           '--dx', LaunchConfiguration('dx').perform(context),
           '--dy', LaunchConfiguration('dy').perform(context),
           '--cog-depth', LaunchConfiguration('cog-depth').perform(context),
           '--thrust-max', LaunchConfiguration('thrust-max').perform(context),
           '--output', LaunchConfiguration('output').perform(context)]
    return [ExecuteProcess(cmd=cmd, output='screen')]


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument(flag, default_value=default, description=text)
        for flag, default, text in ARGUMENTS
    ]
    return LaunchDescription(declarations + [OpaqueFunction(function=_launch_setup)])
