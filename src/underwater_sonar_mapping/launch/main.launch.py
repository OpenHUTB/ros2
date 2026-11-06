from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    save_path = LaunchConfiguration('save_path')
    return LaunchDescription([
        DeclareLaunchArgument(
            'save_path',
            default_value='',
            description='Optional PNG path for saving the rendered sonar scene.',
        ),
        Node(
            package='underwater_sonar_mapping',
            executable='main',
            name='underwater_sonar_mapping_node',
            output='screen',
            parameters=[{'save_path': save_path}],
        ),
    ])