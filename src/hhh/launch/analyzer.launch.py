from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_analyzer',
            executable='analyzer_node', # 对应 setup.py 里的 entry_points
            name='drone_analyzer',
            output='screen'
        )
    ])
