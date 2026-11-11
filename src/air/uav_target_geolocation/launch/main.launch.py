#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动目标地理定位在线判定节点。

    ros2 launch uav_target_geolocation main.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="uav_target_geolocation",
            executable="geo_node",
            name="geo_node",
            output="screen",
            parameters=[
                {"los_bias_deg": [0.0, 0.0],
                 "los_sigma_deg": 0.30,
                 "min_stations_for_fusion": 2,
                 "samples_per_station": 100,
                 "odom_topic": "/uav/odom",
                 "los_topic": "/gimbal/los",
                 "ground_topic": "/target/ground",
                 "diag_topic": "/target/geo_diag",
                 "new_station_topic": "/target/geo/new_station",
                 "ground_z": 0.0},
            ],
        ),
    ])
