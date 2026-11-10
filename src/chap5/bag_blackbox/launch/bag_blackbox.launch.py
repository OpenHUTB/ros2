from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bag_blackbox',
            executable='image_record_node.py',
            name='image_record_node',
            output='screen',
            parameters=[
                {'img_save_path': './img_save/'}
            ]
        )
    ])
