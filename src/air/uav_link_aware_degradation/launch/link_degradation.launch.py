#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分级降级监督 + 链路仿真联合启动。

    ros2 launch uav_link_aware_degradation link_degradation.launch.py

话题约定：
  /pilot/cmd_vel            地面站遥控指令（外部节点发布）
  /link_emulator/out_cmd     损伤后的遥控指令
  /uav/cmd_vel               监督节点输出（与 uav_mujoco_sim 桥接一致）
  /uav_link_degradation/state  状态机 JSON
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("uav_link_aware_degradation")
    cfg = os.path.join(pkg, "config", "link_profiles.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="uav_link_aware_degradation",
            executable="link_emulator",
            name="link_emulator",
            output="screen",
            parameters=[cfg, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
        Node(
            package="uav_link_aware_degradation",
            executable="supervisor",
            name="degradation_supervisor",
            output="screen",
            parameters=[cfg, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            remappings=[
                ("~/odom", "/uav/odom"),
                ("~/link_status", "/link_emulator/status"),
                ("~/remote_cmd", "/link_emulator/out_cmd"),
                ("~/cmd_vel_out", "/uav/cmd_vel"),
            ],
        ),
    ])
