#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_slam_nav 的 ROS2 launch（Humble）。

启动：
  - odom_tf：把 /uav/odom 转成 odom->base_link TF
  - points_to_scan：世界系点云 -> /scan
  - slam_toolbox：建图，发 map->odom
  - nav2 bringup：导航栈
  - simple_nav：nav2 失败时的兜底
  - nav_relay：/uav/cmd_vel 唯一发布者
  - 静态 world->odom 单位变换：bridge 的点云 header.frame_id 是 world，接到标准树

前置：Noetic 那边已经跑起 carlair_ros_bridge，并且 ros1_bridge 已把
/lidar/points、/uav/odom、/uav/cmd_vel 三个话题跨到 Humble。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("uav_slam_nav")
    slam_params = os.path.join(pkg_share, "config", "slam_toolbox.yaml")
    nav2_params = os.path.join(pkg_share, "config", "nav2_params.yaml")

    our_nodes = [
        Node(package="uav_slam_nav", executable="odom_tf", name="odom_tf", output="screen"),
        Node(package="uav_slam_nav", executable="points_to_scan", name="points_to_scan", output="screen"),
        Node(package="uav_slam_nav", executable="simple_nav", name="simple_nav", output="screen"),
        Node(package="uav_slam_nav", executable="nav_relay", name="nav_relay", output="screen"),
        # bridge 的点云在 world 系，把它接到 odom（AirSim 的 odom 本身没有回环校正）
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="world_to_odom",
             arguments=["0", "0", "0", "0", "0", "0", "odom", "world"]),
    ]

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("slam_toolbox"),
                         "launch", "online_async_launch.py")),
        launch_arguments={"slam_params_file": slam_params, "use_sim_time": "false"}.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"),
                         "launch", "navigation_launch.py")),
        launch_arguments={"params_file": nav2_params, "use_sim_time": "false"}.items(),
    )

    return LaunchDescription(our_nodes + [GroupAction([slam, nav2])])
