from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_track_recognizer',
            executable='uav_track',
            name='main_uav_track',
            output='screen'
        )
    ])
