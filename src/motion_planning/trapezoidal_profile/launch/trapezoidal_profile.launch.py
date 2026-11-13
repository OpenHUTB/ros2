#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 launch：一次起两个节点，构成"仿真器 + 控制"的完整闭环。

    sim_node（模拟器/传感器） --odom--> plan_control_node（规划+控制） --cmd_vel--> sim_node

用法：

    ros2 launch trapezoidal_profile trapezoidal_profile.launch.py
""" 

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("x0", default_value="0.0",
                              description="模拟器起点 x (m)"),
        DeclareLaunchArgument("distance", default_value="10.0",
                              description="目标位移 (m)"),
        DeclareLaunchArgument("vmax", default_value="1.5",
                              description="最大速度 (m/s)"),
        DeclareLaunchArgument("use_scurve", default_value="false",
                              description="是否用 S 曲线"),
        # 仿真器一侧：接收 cmd_vel，按运动学积分，发布 odom
        Node(
            package="trapezoidal_profile",
            executable="sim_node.py",
            name="kinematic_sim",
            output="screen",
            parameters=[
                {
                    "x0": ParameterValue(LaunchConfiguration("x0"), value_type=float),
                }
            ],
        ),
        # 规划/控制一侧：订阅 odom，发布 cmd_vel 与 planning/path
        Node(
            package="trapezoidal_profile",
            executable="plan_control_node.py",
            name="profile_control",
            output="screen",
            parameters=[
                {
                    "distance": ParameterValue(LaunchConfiguration("distance"), value_type=float),
                    "vmax": ParameterValue(LaunchConfiguration("vmax"), value_type=float),
                    "use_scurve": ParameterValue(LaunchConfiguration("use_scurve"), value_type=bool),
                }
            ],
        ),
    ])
