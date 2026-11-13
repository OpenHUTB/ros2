#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 launch：一次起两个节点，构成"仿真器 + 控制"的完整闭环。

    sim_node（模拟器/传感器） --odom--> plan_control_node（Bug2 导航+控制） --cmd_vel--> sim_node

用法：

    ros2 launch bug2_navigation bug2_navigation.launch.py
    ros2 launch bug2_navigation bug2_navigation.launch.py scenario:=walls
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("x0", default_value="2.0",
                              description="模拟器起点 x (m)"),
        DeclareLaunchArgument("y0", default_value="2.0",
                              description="模拟器起点 y (m)"),
        DeclareLaunchArgument("yaw0", default_value="0.7854",
                              description="模拟器初始朝向 (rad)"),
        DeclareLaunchArgument("v", default_value="2.0",
                              description="巡航速度 (m/s)"),
        DeclareLaunchArgument("scenario", default_value="standard",
                              description="场景：standard / walls（窄门腔，确定性不可达）"),
        # 仿真器一侧：接收 cmd_vel，按运动学积分，发布 odom
        Node(
            package="bug2_navigation",
            executable="sim_node.py",
            name="kinematic_sim",
            output="screen",
            parameters=[
                {
                    "x0": ParameterValue(LaunchConfiguration("x0"), value_type=float),
                    "y0": ParameterValue(LaunchConfiguration("y0"), value_type=float),
                    "yaw0": ParameterValue(LaunchConfiguration("yaw0"), value_type=float),
                }
            ],
        ),
        # 导航/控制一侧：启动时离线执行一次 Bug2 状态机得参考轨迹，
        # 订阅 odom，发布 cmd_vel 与 planning/path
        Node(
            package="bug2_navigation",
            executable="plan_control_node.py",
            name="bug2_planner_control",
            output="screen",
            parameters=[
                {
                    "v": ParameterValue(LaunchConfiguration("v"), value_type=float),
                    "scenario": ParameterValue(LaunchConfiguration("scenario"),
                                                value_type=str),
                }
            ],
        ),
    ])
