import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('project_root',default_value=os.getcwd()),
        SetEnvironmentVariable('PAYLOAD_PROJECT_ROOT',LaunchConfiguration('project_root')),
        Node(package='payload_mass_estimation',executable='payload_estimator',output='screen'),
        Node(package='payload_mass_estimation',executable='payload_replay',output='screen')])
