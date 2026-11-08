from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    share = FindPackageShare('forklift_slam_nav')
    params = PathJoinSubstitution([share, 'config', 'nav2.yaml'])
    slam_params = PathJoinSubstitution([share, 'config', 'slam.yaml'])
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])),
        launch_arguments={'params_file': params, 'use_sim_time': 'false'}.items())
    return LaunchDescription([
        Node(package='forklift_slam_nav', executable='mujoco_robot', output='screen'),
        Node(package='slam_toolbox', executable='sync_slam_toolbox_node', name='slam_toolbox',
             parameters=[slam_params], output='screen'),
        nav2,
    ])
