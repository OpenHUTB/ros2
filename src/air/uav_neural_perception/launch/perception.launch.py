from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('uav_neural_perception')
    return LaunchDescription([
        Node(
            package='uav_neural_perception',
            executable='perception',
            name='uav_neural_perception',
            output='screen',
            parameters=[os.path.join(share, 'config', 'perception.yaml')],
        ),
    ])
