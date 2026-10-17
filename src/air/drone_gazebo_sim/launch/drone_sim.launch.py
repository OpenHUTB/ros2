import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('drone_gazebo_sim')
    world_file = os.path.join(pkg_share, 'worlds', 'drone_world.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'simple_quad.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen')

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen')

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'simple_quad', '-topic', 'robot_description'],
        output='screen')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}])

    bridge = Node(
        package='drone_gazebo_sim',
        executable='drone_bridge',
        output='screen')

    perception = Node(
        package='drone_gazebo_sim',
        executable='drone_perception',
        output='screen')

    return LaunchDescription([
        gzserver, gzclient, spawn_entity, robot_state_publisher, bridge, perception
    ])
