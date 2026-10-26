from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='holoocean_static_camera_yolo_water_fire_detect',
            executable='fire_detector',
            name='fire_detector_node',
            output='screen'
        )
    ])
