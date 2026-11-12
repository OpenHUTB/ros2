# -*- coding: utf-8 -*-
"""启动重定时节点。

    ros2 launch joint_traj_retime main.launch.py
    ros2 launch joint_traj_retime main.launch.py default_v_max:=3.0 default_a_max:=12.0

另开终端看效果：
    ros2 topic echo /joint_trajectory_retimed
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("joint_traj_retime")
    params = os.path.join(share, "config", "params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("default_v_max", default_value="2.0",
                              description="关节速度上限 rad/s"),
        DeclareLaunchArgument("default_a_max", default_value="8.0",
                              description="关节加速度上限 rad/s²"),
        DeclareLaunchArgument("speed_scale", default_value="1.0",
                              description="额外保守系数"),
        Node(
            package="joint_traj_retime",
            executable="retime_node",
            name="joint_traj_retime_node",
            output="screen",
            parameters=[params, {
                "default_v_max": LaunchConfiguration("default_v_max"),
                "default_a_max": LaunchConfiguration("default_a_max"),
                "speed_scale": LaunchConfiguration("speed_scale"),
            }],
        ),
    ])
