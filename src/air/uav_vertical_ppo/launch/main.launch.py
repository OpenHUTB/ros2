"""启动四旋翼垂直通道 PPO 定高控制节点."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """返回模块的默认启动描述."""
    return LaunchDescription([Node(package='uav_vertical_ppo', executable='node', output='screen')])
