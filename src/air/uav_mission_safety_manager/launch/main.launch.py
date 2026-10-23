from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_airsim', default_value='false'),
        DeclareLaunchArgument('airsim_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('airsim_port', default_value='41451'),
        Node(package='uav_mission_safety_manager', executable='mission_manager', output='screen',
             parameters=[os.path.join(get_package_share_directory('uav_mission_safety_manager'),
                                      'config', 'mission.yaml'),
                         {'use_airsim': LaunchConfiguration('use_airsim'),
                          'airsim_host': LaunchConfiguration('airsim_host'),
                          'airsim_port': LaunchConfiguration('airsim_port')}])
    ])
