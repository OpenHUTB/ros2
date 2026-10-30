"""Top-level launch entry for the lidar_obstacle_nn module.

Brings up everything needed for a complete demonstration:

    1. The Gazebo Fortress simulation world (static obstacle_world.sdf
       by default, or world:=dynamic for the controllable slide world)
       exposing a robot with a 2D LiDAR.
    2. The ros_gz bridge forwarding Gazebo /scan to ROS 2 /scan.
    3. The lidar_obstacle_nn inference node that loads the trained MLP
       and publishes /obstacle_class + /obstacle_confidence.

Run with:

    ros2 launch lidar_obstacle_nn main.launch.py

On a machine without a physical display, wrap it in a virtual X server
(the gpu_lidar sensor needs a rendering context to produce echoes):

    xvfb-run -a ros2 launch lidar_obstacle_nn main.launch.py
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE = 'lidar_obstacle_nn'
PACKAGE_SHARE = get_package_share_directory(PACKAGE)


def generate_launch_description() -> LaunchDescription:
    # --- Arguments ----------------------------------------------------
    world = LaunchConfiguration('world')
    model_path = LaunchConfiguration('model_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gpu = LaunchConfiguration('use_gpu')

    default_model = os.path.join(PACKAGE_SHARE, 'models', 'lidar_mlp.pt')

    # --- Gazebo + bridge (delegated to sim.launch.py) ----------------
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(PACKAGE_SHARE, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'world': world,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # --- Inference node ----------------------------------------------
    inference_node = Node(
        package='lidar_obstacle_nn',
        executable='inference_node',
        name='lidar_obstacle_nn_inference',
        output='screen',
        parameters=[
            {
                'model_path': model_path,
                'input_size': 360,
                'danger_distance': 1.0,
                'far_distance': 3.0,
                'use_gpu': use_gpu,
                'use_sim_time': use_sim_time,
            },
        ],
        remappings=[
            ('/obstacle_class', '/obstacle_class'),
            ('/obstacle_confidence', '/obstacle_confidence'),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world',
                default_value='static',
                description="Simulation world: 'static' or 'dynamic'.",
            ),
            DeclareLaunchArgument(
                'model_path',
                default_value=default_model,
                description='Path to the trained lidar_mlp.pt weights.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use simulation clock.',
            ),
            DeclareLaunchArgument(
                'use_gpu',
                default_value='false',
                description=(
                    'Use CUDA for inference if available. Default false '
                    'because the MLP is small and CPU inference is fast; '
                    'also avoids CUDA capability-mismatch issues with '
                    'very recent GPUs.'
                ),
            ),
            sim_launch,
            inference_node,
        ]
    )
