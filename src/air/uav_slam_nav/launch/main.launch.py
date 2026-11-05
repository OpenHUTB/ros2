# Copyright 2026 OpenHUTB
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# See http://www.apache.org/licenses/LICENSE-2.0 for the full text.
"""uav_slam_nav 的启动文件：一条 launch 拉起感知-建图-规划-控制全链路。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# (节点名, --role 取值)：节点名同时决定 params.yaml 里的参数段
NODE_ROLES = (
    ('airsim_source', 'source'),
    ('mapping', 'mapping'),
    ('planner', 'planner'),
    ('controller', 'controller'),
)


def _source_overrides(airsim_ip, airsim_port, vehicle_name):
    """airsim_source 的 IP/端口/载具名覆盖项（其余节点不传）。"""
    return {
        'airsim_ip': ParameterValue(airsim_ip, value_type=str),
        'airsim_port': ParameterValue(airsim_port, value_type=int),
        'vehicle_name': ParameterValue(vehicle_name, value_type=str),
    }


def generate_launch_description():
    """构造 LaunchDescription。"""
    package_share = get_package_share_directory('uav_slam_nav')
    params_file = LaunchConfiguration('params_file')
    airsim_ip = LaunchConfiguration('airsim_ip')
    airsim_port = LaunchConfiguration('airsim_port')
    vehicle_name = LaunchConfiguration('vehicle_name')

    nodes = [
        Node(
            package='uav_slam_nav',
            executable='main',
            name=name,
            namespace='uav',
            arguments=['--role', role],
            parameters=[params_file] + (
                [_source_overrides(airsim_ip, airsim_port, vehicle_name)]
                if name == 'airsim_source' else []),
            output='screen',
            emulate_tty=True,
        )
        for name, role in NODE_ROLES
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=os.path.join(package_share, 'config', 'params.yaml'),
                description='参数文件路径，默认使用包内 config/params.yaml',
            ),
            DeclareLaunchArgument(
                'airsim_ip', default_value='10.0.2.2',
                description='AirSim 所在主机地址，VirtualBox NAT 下宿主机为 10.0.2.2'),
            DeclareLaunchArgument(
                'airsim_port', default_value='41451',
                description='AirSim RPC 端口（settings.json 的 ApiServerPort）'),
            DeclareLaunchArgument(
                'vehicle_name', default_value='',
                description='AirSim 载具名，默认载具留空'),
        ]
        + nodes
    )
