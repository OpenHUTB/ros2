from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='auv_end2end',
            executable='auv_rect_cruise.py',
            name='auv_rect_cruise_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])

