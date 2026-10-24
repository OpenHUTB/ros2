from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='humanoid_zmp_walk_planner',
            executable='main',
            name='humanoid_zmp_walk_planner_node',
            output='screen'
        )
    ])
