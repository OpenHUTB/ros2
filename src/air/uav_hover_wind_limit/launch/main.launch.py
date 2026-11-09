#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""悬停抗风核算的 ROS 2 启动入口.

    ros2 launch uav_hover_wind_limit main.launch.py
    ros2 launch uav_hover_wind_limit main.launch.py max_tilt_deg:=30.0
    ros2 launch uav_hover_wind_limit main.launch.py available_thrust_to_weight:=1.20

节点按参数算一次风包线，之后每 ``period_s`` 秒把 JSON 结果发布到 ``uav/wind_limit``
（``std_msgs/String``，transient local，晚加入的订阅者也能立即拿到）。
"""
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = 'uav_hover_wind_limit'

ARGUMENTS = (
    ('mass', '2.0', '整机质量 kg'),
    ('drag_area', '0.15', '阻力面积 Cd*A, m^2'),
    ('max_tilt_deg', '25.0', '最大倾角 deg'),
    ('altitude_m', '0.0', '海拔 m, 用于换算空气密度'),
    ('available_thrust_to_weight', '0.0', '实际可用推重比, 0 表示未提供'),
    ('period_s', '5.0', '发布周期 s'),
)


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument(name, default_value=default, description=text)
        for name, default, text in ARGUMENTS
    ]
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=float)
        for name, _, _ in ARGUMENTS
    }
    node = Node(
        package=PACKAGE,
        executable='wind_limit_node',
        name=PACKAGE,
        output='screen',
        parameters=[parameters],
    )
    return launch.LaunchDescription(declarations + [node])
