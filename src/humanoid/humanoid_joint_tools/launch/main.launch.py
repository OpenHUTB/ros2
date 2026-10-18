import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('humanoid_joint_tools')
    
    urdf_file = os.path.join(pkg_path, 'urdf', 'simple_humanoid.urdf')
    rviz_file = os.path.join(pkg_path, 'rviz', 'demo.rviz')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
            output='screen'
        ),
        Node(
            package='humanoid_joint_tools',
            executable='mock',
            name='mock_driver',
            output='screen'
        ),
        Node(
            package='humanoid_joint_tools',
            executable='record',
            name='joint_recorder',
            output='screen'
        ),
        Node(
            package='humanoid_joint_tools',
            executable='play',
            name='joint_player',
            output='screen'
        )
    ])
