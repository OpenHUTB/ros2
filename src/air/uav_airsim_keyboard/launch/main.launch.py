# -*- coding: utf-8 -*-
"""launch 启动文件：ros2 launch uav_airsim_keyboard main.launch.py

launch 参数：
    airsim_ip        宿主机 AirSim 的 IP（VirtualBox NAT 下为 10.0.2.2）
    airsim_port      AirSim RPC 端口，默认 41451
    control_rate     速度指令频率，默认 10.0
    pose_rate        位姿发布频率，默认 20.0
    enable_keyboard  是否在进程内启动键盘（launch 下 stdin 不是终端，会自动跳过）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument('airsim_ip', default_value='10.0.2.2',
                              description='AirSim 所在主机 IP'),
        DeclareLaunchArgument('airsim_port', default_value='41451',
                              description='AirSim RPC 端口'),
        DeclareLaunchArgument('vehicle_name', default_value='',
                              description='AirSim 载具名，默认载具留空'),
        DeclareLaunchArgument('control_rate', default_value='10.0',
                              description='速度指令下发频率 (Hz)'),
        DeclareLaunchArgument('pose_rate', default_value='20.0',
                              description='位姿发布频率 (Hz)'),
        DeclareLaunchArgument('enable_keyboard', default_value='true',
                              description='是否在进程内启动键盘读取'),
    ]

    node = Node(
        package='uav_airsim_keyboard',
        executable='main',
        name='uav_airsim_keyboard',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'airsim_ip': LaunchConfiguration('airsim_ip'),
            'airsim_port': ParameterValue(LaunchConfiguration('airsim_port'),
                                         value_type=int),
            'vehicle_name': ParameterValue(LaunchConfiguration('vehicle_name'),
                                           value_type=str),
            'control_rate': ParameterValue(LaunchConfiguration('control_rate'),
                                          value_type=float),
            'pose_rate': ParameterValue(LaunchConfiguration('pose_rate'),
                                       value_type=float),
            'enable_keyboard': ParameterValue(LaunchConfiguration('enable_keyboard'),
                                             value_type=bool),
        }],
    )

    return LaunchDescription(args + [node])
