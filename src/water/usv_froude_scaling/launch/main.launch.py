#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动核算的离线入口（ExecuteProcess, 无需 rclpy）.

    ros2 launch usv_froude_scaling main.launch.py

入口用 `python -m usv_froude_scaling`：ament_python 把 console script 装到
`install/<pkg>/lib/<pkg>/`，`source install/setup.bash` 不会把该目录加进 PATH，
直接用裸脚本名会 FileNotFoundError；而 `python -m` 只依赖 PYTHONPATH，
setup.bash 一定把它指向本工作区（也避免 PATH 上同名旧脚本被误用）。
"""
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

PACKAGE = 'usv_froude_scaling'

ARGUMENTS = (
    ('speed', '5.0', '实船航速 m/s'),
    ('length', '10.0', '实船水线长 m'),
    ('scale', '10.0', '缩尺比 lambda'),
    ('wetted-area', '22.0', '实船湿表面积 m^2'),
    ('model-resistance', '2.5', '船模实测总阻力 N'),
    ('efficiency', '0.55', '准推进效率'),
    ('rho', '1025.0', '水密度 kg/m^3'),
    ('nu', '0.00000119', '运动黏度 m^2/s'),
    ('output', 'outputs', '输出目录'),
)


def _launch_setup(context):
    """按实际参数拼出命令行."""
    cmd = [sys.executable, '-m', PACKAGE,
        '--speed', LaunchConfiguration('speed').perform(context),
        '--length', LaunchConfiguration('length').perform(context),
        '--scale', LaunchConfiguration('scale').perform(context),
        '--wetted-area', LaunchConfiguration('wetted-area').perform(context),
        '--model-resistance', LaunchConfiguration('model-resistance').perform(context),
        '--efficiency', LaunchConfiguration('efficiency').perform(context),
        '--rho', LaunchConfiguration('rho').perform(context),
        '--nu', LaunchConfiguration('nu').perform(context),
           '--output', LaunchConfiguration('output').perform(context)]
    return [ExecuteProcess(cmd=cmd, output='screen')]


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument(flag, default_value=default, description=text)
        for flag, default, text in ARGUMENTS
    ]
    return LaunchDescription(declarations + [OpaqueFunction(function=_launch_setup)])
