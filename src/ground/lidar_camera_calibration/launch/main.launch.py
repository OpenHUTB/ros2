#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键启动相机-激光雷达外参标定节点。

    ros2 launch lidar_camera_calibration main.launch.py
    ros2 launch lidar_camera_calibration main.launch.py views:=4
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    views = LaunchConfiguration('views')
    pixel_noise = LaunchConfiguration('pixel_noise')
    range_noise = LaunchConfiguration('range_noise')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    return LaunchDescription([
        DeclareLaunchArgument('views', default_value='3',
                              description='标定板摆放次数（1..6）'),
        DeclareLaunchArgument('pixel_noise', default_value='1.0',
                              description='像素噪声标准差 (px)'),
        DeclareLaunchArgument('range_noise', default_value='0.01',
                              description='激光测距噪声标准差 (m)'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera_info',
                              description='相机内参话题'),
        Node(
            package='lidar_camera_calibration',
            executable='calibration_node',
            name='lidar_camera_calibration',
            output='screen',
            parameters=[{
                'views': views,
                'pixel_noise': pixel_noise,
                'range_noise': range_noise,
                'camera_info_topic': camera_info_topic,
            }],
        ),
    ])
