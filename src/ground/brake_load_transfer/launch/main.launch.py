# -*- coding: utf-8 -*-
"""启动制动节点。

    ros2 launch brake_load_transfer main.launch.py
    ros2 launch brake_load_transfer main.launch.py payload:=300.0 payload_x:=2.4

另开终端给踏板：
    ros2 topic pub --times 1 /vehicle/brake_cmd std_msgs/msg/Float64 "{data: 1.0}"
    ros2 topic echo /vehicle/brake_state
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("brake_load_transfer")
    params = os.path.join(share, "config", "params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("payload", default_value="0.0",
                              description="载重 kg"),
        DeclareLaunchArgument("payload_x", default_value="1.35",
                              description="载重质心距前轴 m"),
        DeclareLaunchArgument("brake_bias", default_value="0.62",
                              description="前轴制动力占比"),
        DeclareLaunchArgument("mu", default_value="0.90",
                              description="路面附着系数"),
        Node(
            package="brake_load_transfer",
            executable="brake_node",
            name="brake_load_transfer_node",
            output="screen",
            parameters=[params, {
                "payload": LaunchConfiguration("payload"),
                "payload_x": LaunchConfiguration("payload_x"),
                "brake_bias": LaunchConfiguration("brake_bias"),
                "mu": LaunchConfiguration("mu"),
            }],
        ),
    ])
