from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='usv_colregs_avoidance', executable='main',
             name='usv_colregs_avoidance', output='screen',
             arguments=['--node']),
    ])
