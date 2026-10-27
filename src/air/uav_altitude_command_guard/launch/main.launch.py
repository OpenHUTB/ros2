"""启动无人机定高指令限幅与超时保持节点."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """返回模块的默认启动描述."""
    return LaunchDescription([Node(package='uav_altitude_command_guard', executable='node', output='screen')])
