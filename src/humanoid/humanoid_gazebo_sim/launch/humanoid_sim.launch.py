#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('humanoid_gazebo_sim')
    world = os.path.join(pkg, 'worlds', 'humanoid_world.world')
    return LaunchDescription([
        ExecuteProcess(cmd=['gazebo', '--verbose', world], output='screen'),
        Node(package='humanoid_gazebo_sim', executable='humanoid_teleop', name='humanoid_teleop', output='screen'),
        Node(package='humanoid_gazebo_sim', executable='humanoid_perception', name='humanoid_perception', output='screen'),
        Node(package='humanoid_gazebo_sim', executable='humanoid_controller', name='humanoid_controller', output='screen'),
        Node(package='humanoid_gazebo_sim', executable='humanoid_bridge', name='humanoid_bridge', output='screen'),
    ])
