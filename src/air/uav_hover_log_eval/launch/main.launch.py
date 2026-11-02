"""启动无人机定高日志误差与恢复时间评估节点."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """返回模块的默认启动描述."""
    return LaunchDescription([Node(package='uav_hover_log_eval', executable='node', output='screen')])
