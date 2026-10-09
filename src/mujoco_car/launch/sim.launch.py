from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mujoco_car',
            executable='sim_node',
            output='screen'
        )
    ])