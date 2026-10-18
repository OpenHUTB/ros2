# -*- coding: utf-8 -*-
"""ROS 2 Humble 启动入口。

用法::

    ros2 launch carla_vehicle_dynamics_test main.launch.py host:=192.168.148.1
    ros2 launch carla_vehicle_dynamics_test main.launch.py host:=192.168.148.1 \
        mode:=accel vehicle:=vehicle.bmw.grandtourer v_target_kmh:=100

注意：节点内所有参数都以**字符串**声明，因此这里必须用
``ParameterValue(..., value_type=str)`` 强制按字符串传递。否则 launch_ros 会按默认值
"猜"类型（例如把 ``'2000'`` 猜成 INTEGER、把 ``'60.0'`` 猜成 DOUBLE），
与节点声明的 STRING 冲突，节点会立刻退出并报参数类型不匹配。
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
    ('mode', 'all', '测试项：accel / brake / turn / all'),
    ('vehicle', 'vehicle.tesla.model3', '单个车型 blueprint'),
    ('vehicles', 'all', '写 all 则循环内置多车型对比；留空则只测 vehicle'),
    ('v_target_kmh', '60.0', '目标车速（km/h）'),
    ('steer', '0.05', '阶跃转向的归一化转向指令（-1~1）'),
    ('turn_speed', '7.0', '转向测试的目标车速（m/s）'),
    ('turn_time', '1.5', '转向段的持续时间（秒）'),
    ('max_time', '40.0', '单项测试超时（秒）'),
    ('log_every', '1.0', '日志打印间隔（秒）'),
    ('out_dir', 'data', 'CSV 与曲线图输出目录'),
)


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in ARGUMENTS
    ]

    node = Node(
        package='carla_vehicle_dynamics_test',
        executable='dynamics_test_node',
        name='dynamics_test_node',
        output='screen',
        parameters=[{name: ParameterValue(LaunchConfiguration(name), value_type=str)
                     for name, _, _ in ARGUMENTS}],
    )

    return LaunchDescription(declared_arguments + [node])
