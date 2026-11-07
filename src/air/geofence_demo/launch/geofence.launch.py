from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='geofence_demo',
            executable='geofence_node',
            name='geofence_node',
            output='screen'
        )
    ])
