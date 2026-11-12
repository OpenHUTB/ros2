# -*- coding: utf-8 -*-
"""ROS 2 Humble 启动入口。

用法::

    ros2 launch carla_visual_lane_departure_warning main.launch.py
    ros2 launch carla_visual_lane_departure_warning main.launch.py \
        host:=192.168.148.1 checkpoint:=models/lane_regressor.npz

节点里所有参数都按**字符串**声明，所以这里必须用
``ParameterValue(..., value_type=str)`` 传，否则 launch_ros 会把 ``'2000'``
猜成 INTEGER，与节点声明的 STRING 冲突，节点会直接退出。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGUMENTS = (
    ('checkpoint', 'models/lane_regressor.npz', '训练好的权重路径'),
    ('host', '127.0.0.1', 'Windows 宿主机 IP（VMware NAT 下是 VMnet8 的地址）'),
    ('port', '2000', 'CARLA RPC 端口'),
    ('town', 'Town03', 'CARLA 地图'),
    ('vehicle', 'vehicle.tesla.model3', '自车 blueprint'),
    ('spawn_index', '0', '出生点索引'),
    ('dt', '0.05', '同步步长（秒）'),
    ('frames', '0', '跑多少帧后退出，0 表示一直跑'),
)


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in ARGUMENTS
    ]

    node = Node(
        package='carla_visual_lane_departure_warning',
        executable='visual_lane_departure_node',
        name='visual_lane_departure_warning',
        output='screen',
        parameters=[{name: ParameterValue(LaunchConfiguration(name), value_type=str)
                     for name, _, _ in ARGUMENTS}],
    )

    return LaunchDescription(declared_arguments + [node])
