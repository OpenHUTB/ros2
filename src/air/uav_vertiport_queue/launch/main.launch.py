#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动在线排队节点（订阅各机请求，发布排队序列与预计等待时间）。"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = os.path.join(get_package_share_directory("uav_vertiport_queue"),
                          "config", "uav_vertiport_queue.yaml")
    return LaunchDescription([
        Node(
            package="uav_vertiport_queue",
            executable="queue_node",
            name="uav_vertiport_queue",
            output="screen",
            parameters=[config],
        ),
    ])
