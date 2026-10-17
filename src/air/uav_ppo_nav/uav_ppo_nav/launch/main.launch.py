from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_ppo_nav',
            executable='ppo_nav_node',
            name='ppo_nav_node'
        )
    ])
