from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rov_mujoco_slam',
            executable='main_sim_node.py',
            name='main_sim_node',
            output='screen'
        ),
        Node(
            package='rov_mujoco_slam',
            executable='sonar_slam_node.py',
            name='sonar_slam_node',
            output='screen'
        )
    ])