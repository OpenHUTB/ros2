#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动核算的离线入口（ExecuteProcess, 无需 rclpy）.

    ros2 launch diff_drive_powertrain_sizing main.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

PACKAGE = 'diff_drive_powertrain_sizing'

ARGUMENTS = (
    ('mass', '12.0', '整车质量 kg'),
    ('wheel-radius', '0.06', '轮半径 m'),
    ('driven-wheels', '2', '驱动轮数量'),
    ('grade', '10.0', '坡度 deg'),
    ('accel', '1.0', '目标加速度 m/s^2'),
    ('v-cruise', '1.5', '巡航速度 m/s'),
    ('efficiency', '0.85', '传动效率'),
    ('motor-cont-torque', '1.2', '电机持续转矩 N*m'),
    ('motor-peak-torque', '2.5', '电机峰值转矩 N*m'),
    ('motor-rated-rpm', '4000.0', '电机额定转速 rpm'),
    ('bus-voltage', '24.0', '母线电压 V'),
    ('output', 'outputs', '输出目录'),
)


def _launch_setup(context):
    """按实际参数拼出命令行."""
    cmd = ['diff_drive_powertrain_sizing',
           '--mass', LaunchConfiguration('mass').perform(context),
           '--wheel-radius', LaunchConfiguration('wheel-radius').perform(context),
           '--driven-wheels', LaunchConfiguration('driven-wheels').perform(context),
           '--grade', LaunchConfiguration('grade').perform(context),
           '--accel', LaunchConfiguration('accel').perform(context),
           '--v-cruise', LaunchConfiguration('v-cruise').perform(context),
           '--efficiency', LaunchConfiguration('efficiency').perform(context),
           '--motor-cont-torque', LaunchConfiguration('motor-cont-torque').perform(context),
           '--motor-peak-torque', LaunchConfiguration('motor-peak-torque').perform(context),
           '--motor-rated-rpm', LaunchConfiguration('motor-rated-rpm').perform(context),
           '--bus-voltage', LaunchConfiguration('bus-voltage').perform(context),
           '--output', LaunchConfiguration('output').perform(context)]
    return [ExecuteProcess(cmd=cmd, output='screen')]


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument(flag, default_value=default, description=text)
        for flag, default, text in ARGUMENTS
    ]
    return LaunchDescription(declarations + [OpaqueFunction(function=_launch_setup)])
