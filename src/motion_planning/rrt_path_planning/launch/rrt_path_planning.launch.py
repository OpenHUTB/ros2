#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 launch：一次起两个节点，构成"仿真器 + 控制"的完整闭环。

    sim_node（模拟器/传感器） --odom--> plan_control_node（规划+控制） --cmd_vel--> sim_node

用法：

    ros2 launch rrt_path_planning rrt_path_planning.launch.py
""" 

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("x0", default_value="8.0",
                              description="模拟器起点 x (m)"),
        DeclareLaunchArgument("y0", default_value="8.0",
                              description="模拟器起点 y (m)"),
        DeclareLaunchArgument("yaw0", default_value="0.7854",
                              description="模拟器初始朝向 (rad)"),
        DeclareLaunchArgument("lookahead", default_value="8.0",
                              description="纯跟踪前视距离 (m)"),
        # 仿真器一侧：接收 cmd_vel，按运动学积分，发布 odom
        Node(
            package="rrt_path_planning",
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
        # 规划/控制一侧：订阅 odom，发布 cmd_vel 与 planning/path
        Node(
            package="rrt_path_planning",
            executable="plan_control_node.py",
            name="rrt_planner_control",
            output="screen",
            parameters=[
                {
                    "lookahead": ParameterValue(LaunchConfiguration("lookahead"), value_type=float),
                }
            ],
        ),
    ])
