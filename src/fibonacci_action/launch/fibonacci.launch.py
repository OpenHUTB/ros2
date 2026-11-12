"""
fibonacci_action 一键启动 launch 文件

同时启动 Action 服务端和客户端，演示完整的 Action 通信流程。
客户端目标阶数可通过 order 参数覆盖。

运行方式：
    ros2 launch fibonacci_action fibonacci.launch.py
    ros2 launch fibonacci_action fibonacci.launch.py order:=15
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 可配置客户端目标阶数
    order_arg = DeclareLaunchArgument(
        'order',
        default_value='10',
        description='客户端请求的斐波那契数列目标阶数',
    )

    # Action 服务端节点
    server_node = Node(
        package='fibonacci_action',
        executable='server',
        name='fibonacci_action_server',
        output='screen',
    )

    # Action 客户端节点
    client_node = Node(
        package='fibonacci_action',
        executable='client',
        name='fibonacci_action_client',
        output='screen',
        arguments=[LaunchConfiguration('order')],
    )

    return LaunchDescription([
        order_arg,
        server_node,
        client_node,
    ])
