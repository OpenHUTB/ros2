#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动核算的离线入口（ExecuteProcess, 无需 rclpy）.

    ros2 launch uav_cg_trim_envelope main.launch.py

可执行入口：优先用 PATH 上的 console script `uav_cg_trim_envelope`；找不到时退回
`python -m uav_cg_trim_envelope`（source install/setup.bash 后 PYTHONPATH 已包含本包）。
ament_python 把 console script 装到 `install/<pkg>/lib/<pkg>/`，source install/setup.bash
并不会把该目录加入 PATH，所以必须有这条兜底。
"""
import shutil
import sys

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


def resolve_command(flags):
    """console script 优先，缺失时退回 python -m，两者都调同一个 CLI."""
    executable = shutil.which(PACKAGE)
    if executable:
        return [executable] + flags
    return [sys.executable, '-m', PACKAGE] + flags


def _launch_setup(context):
    """按实际参数拼出命令行."""
    flags = [
        '--mass', LaunchConfiguration('mass').perform(context),
        '--arm-x', LaunchConfiguration('arm-x').perform(context),
        '--arm-y', LaunchConfiguration('arm-y').perform(context),
        '--dx', LaunchConfiguration('dx').perform(context),
        '--dy', LaunchConfiguration('dy').perform(context),
        '--cog-depth', LaunchConfiguration('cog-depth').perform(context),
        '--thrust-max', LaunchConfiguration('thrust-max').perform(context),
        '--output', LaunchConfiguration('output').perform(context),
    ]
    return [ExecuteProcess(cmd=resolve_command(flags), output='screen')]


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument(flag, default_value=default, description=text)
        for flag, default, text in ARGUMENTS
    ]
    return LaunchDescription(declarations + [OpaqueFunction(function=_launch_setup)])
