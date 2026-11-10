# -*- coding: utf-8 -*-
"""ROS 2 Humble 启动入口。

用法::

    ros2 launch carla_lidar_pointcloud_segmentation main.launch.py
    ros2 launch carla_lidar_pointcloud_segmentation main.launch.py \
        host:=192.168.148.1 town:=Town03 corridor_width:=1.5

节点里所有参数都按**字符串**声明，所以这里必须用
``ParameterValue(..., value_type=str)`` 传，否则 launch_ros 会把 ``'2000'``
猜成 INTEGER、把 ``'1.2'`` 猜成 DOUBLE，与节点声明的 STRING 冲突，节点会直接退出。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGUMENTS = (
    ('checkpoint', 'models/seg_mlp.npz', '训练好的权重路径'),
    ('host', '127.0.0.1', 'Windows 宿主机 IP（VMware NAT 下是 VMnet8 的地址）'),
    ('port', '2000', 'CARLA RPC 端口'),
    ('town', '', '地图；留空沿用服务端当前地图，填了才会切图'),
    ('vehicle', 'vehicle.tesla.model3', '自车 blueprint'),
    ('spawn_index', '0', '出生点索引'),
    ('traffic', '12', 'NPC 车数量，0 表示不放'),
    ('lidar_range', '60.0', '激光雷达量程（米）'),
    ('channels', '32', '激光雷达线数'),
    ('points_per_second', '200000', '激光雷达每秒点数'),
    ('rotation', '20.0', '激光雷达转速（Hz）'),
    ('lidar_height', '2.4', '激光雷达安装高度（米）'),
    ('corridor_width', '1.2', '车身走廊半宽（米）'),
    ('comfort_distance', '14.0', '大于该距离全速（米）'),
    ('stop_distance', '5.0', '小于该距离刹停（米）'),
    ('lookahead', '8.0', '纯跟踪预瞄距离（米）'),
    ('max_steer', '0.6', '最大前轮转角（弧度）'),
    ('max_speed', '8.0', '目标车速上限（米/秒）'),
    ('free_window', '5', '前方余量的中值滤波帧数，1 表示不过滤'),
)


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in ARGUMENTS
    ]

    node = Node(
        package='carla_lidar_pointcloud_segmentation',
        executable='lidar_segmentation_node',
        name='lidar_segmentation_node',
        output='screen',
        parameters=[{name: ParameterValue(LaunchConfiguration(name), value_type=str)
                     for name, _, _ in ARGUMENTS}],
    )

    return LaunchDescription(declared_arguments + [node])