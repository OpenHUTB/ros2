import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'mujoco_traffic_light_stop'
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', os.path.join(get_package_share_directory(package_name), 'main.py')],
            output='screen'
        ),
    ])
