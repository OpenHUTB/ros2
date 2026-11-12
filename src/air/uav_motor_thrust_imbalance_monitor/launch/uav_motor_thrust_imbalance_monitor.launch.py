"""ROS 2 launch：电机推力不对称监测节点。

参数与判据门限可在 launch 内覆盖（对照 motor_imbalance_node.py 的
defaults）；无参数时使用判据默认值。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'n_motors', default_value='4',
            description='电机数量（默认四旋翼）'),
        DeclareLaunchArgument(
            'rpm_max', default_value='8000.0',
            description='指令 1.0 对应的空载转速（RPM）'),
        DeclareLaunchArgument(
            'stale_timeout', default_value='1.0',
            description='遥测超时门限（秒）'),
        Node(
            package='uav_motor_thrust_imbalance_monitor',
            executable='motor_imbalance_node',
            name='motor_imbalance_monitor',
            output='screen',
            parameters=[{
                'n_motors': LaunchConfiguration('n_motors'),
                'rpm_max': LaunchConfiguration('rpm_max'),
                'stale_timeout': LaunchConfiguration('stale_timeout'),
            }],
        ),
    ])
