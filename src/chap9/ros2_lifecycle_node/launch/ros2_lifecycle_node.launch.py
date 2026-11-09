#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 launch：起一个生命周期节点。

注意：lifecycle 节点起来后是 unconfigured，什么都不会发布。需要手动触发：

    ros2 lifecycle set /lifecycle_heartbeat configure
    ros2 lifecycle set /lifecycle_heartbeat activate
    ros2 topic echo /heartbeat

也可以把触发写进 launch（用 EmitEvent + ChangeState），但那会掩盖「显式
管理状态」这个教学重点，所以这里留手动。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("period", default_value="0.5",
                              description="心跳周期 (s)"),
        DeclareLaunchArgument("fail_on_configure", default_value="false",
                              description="让 configure 故意失败，演示状态不迁移"),
        Node(
            package="ros2_lifecycle_node",
            executable="ros_node.py",
            name="lifecycle_heartbeat",
            output="screen",
            parameters=[{
                "period": ParameterValue(LaunchConfiguration("period"), value_type=float),
                "fail_on_configure": ParameterValue(
                    LaunchConfiguration("fail_on_configure"), value_type=bool),
            }],
        ),
    ])
