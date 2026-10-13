"""Bring up the Gazebo Fortress simulation and the ros_gz bridge.

This launch file:
    1. Starts ``ign gazebo`` with ``obstacle_world.sdf`` (autostart clock).
    2. Starts ``parameter_bridge`` forwarding Gazebo ``/scan`` to
       ROS 2 ``/scan`` (sensor_msgs/LaserScan).

It is wrapped by ``main.launch.py`` (which also starts the inference
node) and is also runnable alone for simulation-only tests:

    ros2 launch lidar_obstacle_nn sim.launch.py
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


PACKAGE = 'lidar_obstacle_nn'
PACKAGE_SHARE = get_package_share_directory(PACKAGE)


def generate_launch_description() -> LaunchDescription:
    world_path = LaunchConfiguration('world_path')
    headless = LaunchConfiguration('headless')

    default_world = os.path.join(PACKAGE_SHARE, 'worlds', 'obstacle_world.sdf')

    # --- Gazebo Fortress with GUI -----------------------------------
    gz_sim_gui = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '2', world_path],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", headless, "' == 'false'"])
        ),
    )

    # --- Gazebo Fortress headless (for CI / no-display runs) --------
    gz_sim_headless = ExecuteProcess(
        cmd=[
            'ign', 'gazebo', '-r', '-v', '2',
            '--headless-rendering', world_path,
        ],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", headless, "' == 'true'"])
        ),
    )

    # --- ros_gz bridge: /scan only ----------------------------------
    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='scan_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world_path',
                default_value=default_world,
                description='Path to the Gazebo SDF world file.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use simulation clock.',
            ),
            DeclareLaunchArgument(
                'headless',
                default_value='false',
                description='Run Gazebo without a GUI (for CI / headless runs).',
            ),
            gz_sim_gui,
            gz_sim_headless,
            scan_bridge,
        ]
    )
