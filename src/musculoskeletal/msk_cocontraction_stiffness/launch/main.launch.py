#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动核算的离线入口（ExecuteProcess, 无需 rclpy）.

    ros2 launch msk_cocontraction_stiffness main.launch.py

入口用 `python -m msk_cocontraction_stiffness`：ament_python 把 console script 装到
`install/<pkg>/lib/<pkg>/`，`source install/setup.bash` 不会把该目录加进 PATH，
直接用裸脚本名会 FileNotFoundError；而 `python -m` 只依赖 PYTHONPATH，
setup.bash 一定把它指向本工作区（也避免 PATH 上同名旧脚本被误用）。
"""
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

PACKAGE = 'msk_cocontraction_stiffness'

ARGUMENTS = (
    ('torque', '8.0', '目标净力矩 N*m'),
    ('moment-arm', '0.03', '肌力臂 m'),
    ('f-max', '600.0', '最大等长力 N'),
    ('slack-length', '0.25', '肌肉静息长度 m'),
    ('q-factor', '25.0', '肌刚度系数 q'),
    ('coactivation', '0.3', '共收缩水平 [0,1]'),
    ('disturbance', '2.0', '扰动力矩 N*m'),
    ('output', 'outputs', '输出目录'),
)


def _launch_setup(context):
    """按实际参数拼出命令行."""
    cmd = [sys.executable, '-m', PACKAGE,
        '--torque', LaunchConfiguration('torque').perform(context),
        '--moment-arm', LaunchConfiguration('moment-arm').perform(context),
        '--f-max', LaunchConfiguration('f-max').perform(context),
        '--slack-length', LaunchConfiguration('slack-length').perform(context),
        '--q-factor', LaunchConfiguration('q-factor').perform(context),
        '--coactivation', LaunchConfiguration('coactivation').perform(context),
        '--disturbance', LaunchConfiguration('disturbance').perform(context),
           '--output', LaunchConfiguration('output').perform(context)]
    return [ExecuteProcess(cmd=cmd, output='screen')]


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument(flag, default_value=default, description=text)
        for flag, default, text in ARGUMENTS
    ]
    return LaunchDescription(declarations + [OpaqueFunction(function=_launch_setup)])
