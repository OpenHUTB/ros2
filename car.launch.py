from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_share = FindPackageShare(package='ultra_avoid').find('ultra_avoid')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'car.urdf'])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf_file],
            output='screen'
        ),
        Node(
            package='ultra_avoid',
            executable='ultra_node',
            output='screen'
        )
    ])
