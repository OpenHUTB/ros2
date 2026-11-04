# -*- coding: utf-8 -*-
"""ROS 2 Humble 启动入口。

用法::

    ros2 launch carla_nn_car_following main.launch.py host:=192.168.148.1 scenario:=decelerate

说明：节点内所有参数都以**字符串**声明，因此这里必须用
``ParameterValue(..., value_type=str)`` 强制按字符串传递。否则 launch_ros 会按默认值
"猜"类型（例如把 ``'2000'`` 猜成 INTEGER、把 ``'60.0'`` 猜成 DOUBLE），
与节点声明的 STRING 冲突，节点会立刻退出并报：

    Trying to set parameter 'port' to '2000' of type 'INTEGER', expecting type 'STRING'
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGUMENTS = (
    ('host', '192.168.148.1', 'Windows 宿主机 IP（ipconfig 查看；VMware NAT 下为 VMnet8 的地址）'),
    ('port', '2000', 'CARLA RPC 端口'),
    ('town', 'Town03', 'CARLA 地图'),
    ('weather', 'ClearNoon', '天气：ClearNoon / HardRainNoon / ClearNight'),
    ('scenario', 'decelerate', '工况：constant / decelerate / emergency / cutin'),
    ('duration', '60.0', '仿真时长（秒）'),
    ('dt', '0.05', '同步步长（秒）'),
    ('lead_distance', '25.0', '前车初始距离（米）'),
    ('controller', 'classic', '控制器：classic = 经典 ACC；nn = 神经网络（需先训练出模型）'),
    ('models_dir', 'models', '神经网络模型目录（controller:=nn 时使用）'),
)


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in ARGUMENTS
    ]

    node = Node(
        package='carla_nn_car_following',
        executable='acc_scene_node',
        name='acc_scene_node',
        output='screen',
        parameters=[{name: ParameterValue(LaunchConfiguration(name), value_type=str)
                     for name, _, _ in ARGUMENTS}],
    )

    return LaunchDescription(declared_arguments + [node])
