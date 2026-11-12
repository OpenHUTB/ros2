#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键启动：合成数据源 + 时间偏移估计节点（无需任何硬件）。

    ros2 launch camera_imu_time_calibration main.launch.py
    ros2 launch camera_imu_time_calibration main.launch.py offset_ms:=120 cam_rate:=15

默认注入 +50 ms 偏移，估计节点启动约 1 s 后开始输出，结果见
``/camera_imu_time_estimator/time_offset``。

所有数值参数都用 ``ParameterValue(..., value_type=float)`` 显式声明类型：
``launch`` 拿到的是字符串，若交给它自己推断，``offset_ms:=120`` 会被猜成 int，
而节点侧按 double 使用，就会抛 ``InvalidParameterTypeException``。这里写死类型，
用户不必记住"要带小数点"。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    offset_ms = LaunchConfiguration('offset_ms')
    cam_rate = LaunchConfiguration('cam_rate')
    imu_topic = LaunchConfiguration('imu_topic')
    cam_topic = LaunchConfiguration('cam_topic')
    window_sec = LaunchConfiguration('window_sec')

    return LaunchDescription([
        DeclareLaunchArgument('offset_ms', default_value='50.0',
                              description='数据源注入的相机时间偏移 (ms)'),
        DeclareLaunchArgument('cam_rate', default_value='30.0',
                              description='相机角速度发布频率 (Hz)'),
        DeclareLaunchArgument('imu_topic', default_value='/imu',
                              description='IMU 话题'),
        DeclareLaunchArgument('cam_topic', default_value='/camera/angular_velocity',
                              description='相机角速度话题'),
        DeclareLaunchArgument('window_sec', default_value='8.0',
                              description='估计窗口长度 (s)'),
        Node(
            package='camera_imu_time_calibration',
            executable='synthetic_source',
            name='camera_imu_synthetic_source',
            output='screen',
            parameters=[{
                'offset_ms': ParameterValue(offset_ms, value_type=float),
                'cam_rate': ParameterValue(cam_rate, value_type=float),
                'imu_topic': ParameterValue(imu_topic, value_type=str),
                'cam_topic': ParameterValue(cam_topic, value_type=str),
            }],
        ),
        Node(
            package='camera_imu_time_calibration',
            executable='estimator_node',
            name='camera_imu_time_estimator',
            output='screen',
            parameters=[{
                'imu_topic': ParameterValue(imu_topic, value_type=str),
                'cam_topic': ParameterValue(cam_topic, value_type=str),
                'window_sec': ParameterValue(window_sec, value_type=float),
            }],
        ),
    ])
