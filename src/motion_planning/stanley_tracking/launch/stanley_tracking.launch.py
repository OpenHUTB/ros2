#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 launch：一次起两个节点，构成"仿真器 + 控制"的完整闭环。

    sim_node（模拟器/传感器） --odom--> plan_control_node（规划+控制） --cmd_vel--> sim_node

用法：

    ros2 launch stanley_tracking stanley_tracking.launch.py
    # 关掉曲率前馈，观察弯道上的稳态偏差（对照实验）
    ros2 launch stanley_tracking stanley_tracking.launch.py feedforward:=false

参数名必须与 plan_control_node 的 declare_parameter("feedforward") 一致，
否则参数传不进节点、对照实验跑不起来；tests/test_stanley.py 有对应的一致性测试。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("x0", default_value="9.5",
                              description="模拟器起点 x (m)"),
        DeclareLaunchArgument("yaw0", default_value="1.5708",
                              description="模拟器初始朝向 (rad)"),
        DeclareLaunchArgument("radius", default_value="8.0",
                              description="参考圆半径 (m)"),
        DeclareLaunchArgument("k", default_value="1.2",
                              description="横向增益"),
        DeclareLaunchArgument("feedforward", default_value="true",
                              description="是否启用曲率前馈（false 为原版 Stanley 对照）"),
        # 仿真器一侧：接收 cmd_vel，按运动学积分，发布 odom
        Node(
            package="stanley_tracking",
            executable="sim_node.py",
            name="kinematic_sim",
            output="screen",
            parameters=[
                {
                    "x0": ParameterValue(LaunchConfiguration("x0"), value_type=float),
                    "yaw0": ParameterValue(LaunchConfiguration("yaw0"), value_type=float),
                }
            ],
        ),
        # 规划/控制一侧：订阅 odom，发布 cmd_vel 与 planning/path
        Node(
            package="stanley_tracking",
            executable="plan_control_node.py",
            name="stanley_control",
            output="screen",
            parameters=[
                {
                    "radius": ParameterValue(LaunchConfiguration("radius"), value_type=float),
                    "k": ParameterValue(LaunchConfiguration("k"), value_type=float),
                    "feedforward": ParameterValue(LaunchConfiguration("feedforward"),
                                                  value_type=bool),
                }
            ],
        ),
    ])
