#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动核算的离线入口（ExecuteProcess, 无需 rclpy）.

    ros2 launch humanoid_support_margin main.launch.py

可执行入口：优先用 PATH 上的 console script `humanoid_support_margin`；找不到时退回
`python -m humanoid_support_margin`（source install/setup.bash 后 PYTHONPATH 已包含本包）。
ament_python 把 console script 装到 `install/<pkg>/lib/<pkg>/`，source install/setup.bash
并不会把该目录加入 PATH，所以必须有这条兜底。
"""
import shutil
import sys

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
        '--foot1-x', LaunchConfiguration('foot1-x').perform(context),
        '--foot2-x', LaunchConfiguration('foot2-x').perform(context),
        '--foot-width', LaunchConfiguration('foot-width').perform(context),
        '--foot-length', LaunchConfiguration('foot-length').perform(context),
        '--com-x', LaunchConfiguration('com-x').perform(context),
        '--com-y', LaunchConfiguration('com-y').perform(context),
        '--com-height', LaunchConfiguration('com-height').perform(context),
        '--output', LaunchConfiguration('output').perform(context),
    ]
    return [ExecuteProcess(cmd=resolve_command(flags), output='screen')]


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument(flag, default_value=default, description=text)
        for flag, default, text in ARGUMENTS
    ]
    return LaunchDescription(declarations + [OpaqueFunction(function=_launch_setup)])
