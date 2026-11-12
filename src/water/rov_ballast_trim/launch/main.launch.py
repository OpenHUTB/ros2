#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动浮力-压载配平核算的离线入口（ExecuteProcess, 无需 rclpy）.

    ros2 launch rov_ballast_trim main.launch.py
    ros2 launch rov_ballast_trim main.launch.py target_gm:=0.05
    ros2 launch rov_ballast_trim main.launch.py beam:=0.80 tray_y:=0.35

可执行入口统一用 `python -m rov_ballast_trim`：ament_python 把 console script 装到
`install/<pkg>/lib/<pkg>/`，`source install/setup.bash` 不会把该目录加入 PATH，
直接用裸脚本名会 FileNotFoundError；而 `python -m` 只依赖 PYTHONPATH，
setup.bash 一定指向本工作区（也避免 PATH 上同名旧脚本被误用）。
"""
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

PACKAGE = 'rov_ballast_trim'

ARGUMENTS = (
    ('length', '0.90', '外壳长度 m (x)'),
    ('beam', '0.70', '外壳宽度 m (y)'),
    ('height', '0.50', '外壳高度 m (z)'),
    ('density', '', '海水密度 kg/m^3, 留空按 10 C/35 psu 表层取 1027'),
    ('tray_x', '0.40', '压载托盘纵向限位 m'),
    ('tray_y', '0.30', '压载托盘横向限位 m'),
    ('tray_z', '-0.18', '压载托盘高度 m'),
    ('added_mass', '1.3', '横摇附加惯量系数'),
    ('target_gm', '', '目标初稳心高 m, 留空不反解安装高度'),
    ('output', 'outputs', '输出目录'),
)


def _launch_setup(context):
    """按实际参数拼出命令行, 留空的可选参数不传递."""
    flags = [
        '--length', LaunchConfiguration('length').perform(context),
        '--beam', LaunchConfiguration('beam').perform(context),
        '--height', LaunchConfiguration('height').perform(context),
        '--tray-x', LaunchConfiguration('tray_x').perform(context),
        '--tray-y', LaunchConfiguration('tray_y').perform(context),
        '--tray-z', LaunchConfiguration('tray_z').perform(context),
        '--added-mass', LaunchConfiguration('added_mass').perform(context),
        '--output', LaunchConfiguration('output').perform(context),
    ]
    for name, flag in (('density', '--density'), ('target_gm', '--target-gm')):
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            flags.extend([flag, value])
    return [ExecuteProcess(cmd=[sys.executable, '-m', PACKAGE] + flags, output='screen')]


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument(name, default_value=default, description=text)
        for name, default, text in ARGUMENTS
    ]
    return LaunchDescription(declarations + [OpaqueFunction(function=_launch_setup)])
