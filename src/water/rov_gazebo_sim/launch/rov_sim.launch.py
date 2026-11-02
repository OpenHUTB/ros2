#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('rov_gazebo_sim')
    world_file = os.path.join(pkg_dir, 'worlds', 'rov_world.world')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file],
            output='screen'),
        Node(
            package='rov_gazebo_sim',
            executable='rov_teleop',
            name='rov_teleop',
            output='screen'),
        Node(
            package='rov_gazebo_sim',
            executable='rov_perception',
            name='rov_perception',
            output='screen'),
        Node(
            package='rov_gazebo_sim',
            executable='rov_controller',
            name='rov_controller',
            output='screen'),
        Node(
            package='rov_gazebo_sim',
            executable='rov_bridge',
            name='rov_bridge',
            output='screen'),
    ])
