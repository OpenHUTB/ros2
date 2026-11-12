# -*- coding: utf-8 -*-
"""ROS 2 Humble 启动入口。

用法::

    ros2 launch carla_auto_light main.launch.py host:=192.168.148.1
    ros2 launch carla_auto_light main.launch.py host:=192.168.148.1 phase_seconds:=4

注意：节点内所有参数都以**字符串**声明，这里必须用
``ParameterValue(..., value_type=str)`` 强制按字符串传递，否则 launch_ros 会按默认值
"猜"类型（例如把 ``'2000'`` 猜成 INTEGER），与节点声明的 STRING 冲突，节点会立刻退出。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGUMENTS = (
    ('host', '127.0.0.1', 'Windows 宿主机 IP（ipconfig 查看；VMware NAT 下为 VMnet8 的地址）'),
    ('port', '2000', 'CARLA RPC 端口'),
    ('town', 'Town03', 'CARLA 地图'),
    ('dt', '0.05', '同步模式步长（秒）'),
    ('vehicle', 'vehicle.tesla.model3', '车型 blueprint'),
    ('spawn_index', '-1', '出生点编号，-1 表示自动挑选'),
    ('phase_seconds', '6.0', '每个天气阶段的时长（秒）'),
)


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in ARGUMENTS
    ]

    node = Node(
        package='carla_auto_light',
        executable='auto_light_node',
        name='auto_light_node',
        output='screen',
        parameters=[{name: ParameterValue(LaunchConfiguration(name), value_type=str)
                     for name, _, _ in ARGUMENTS}],
    )

    return LaunchDescription(declared_arguments + [node])
