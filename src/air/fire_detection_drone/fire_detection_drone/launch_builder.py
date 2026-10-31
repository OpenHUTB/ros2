"""Shared launch description used by both installed launch files."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('fire_detection_drone')
    default_params = os.path.join(
        package_share, 'config', 'fire_detection.yaml'
    )

    arguments = [
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='ROS 2 parameter file for both nodes.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock from the simulator.',
        ),
        DeclareLaunchArgument(
            'start_detector',
            default_value='true',
            description='Start the fire detector node.',
        ),
        DeclareLaunchArgument(
            'start_explorer',
            default_value='true',
            description='Start the drone explorer node.',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/image_raw',
            description='Camera image topic consumed by the detector.',
        ),
        DeclareLaunchArgument(
            'pose_topic',
            default_value='/odom',
            description='Odometry or PoseStamped topic consumed by explorer.',
        ),
        DeclareLaunchArgument(
            'pose_message_type',
            default_value='odometry',
            description='odometry or pose_stamped.',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='Body-frame velocity command topic.',
        ),
        DeclareLaunchArgument(
            'exploration_pattern',
            default_value='lawnmower',
            description='lawnmower, perimeter or spiral.',
        ),
        DeclareLaunchArgument(
            'ned_input',
            default_value='false',
            description='Convert incoming NED pose to ENU.',
        ),
        DeclareLaunchArgument(
            'ned_output',
            default_value='false',
            description='Convert outgoing ENU Twist to NED.',
        ),
        DeclareLaunchArgument(
            'require_odometry',
            default_value='true',
            description='Stop unless fresh pose data is available.',
        ),
        DeclareLaunchArgument(
            'publish_debug_image',
            default_value='true',
            description='Publish annotated camera images.',
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value='',
            description='Optional ONNX fire classifier path.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='ROS 2 log level for both nodes.',
        ),
    ]

    detector_node = Node(
        package='fire_detection_drone',
        executable='fire_detector',
        name='fire_detector',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('start_detector')),
        arguments=[
            '--ros-args',
            '--log-level',
            LaunchConfiguration('log_level'),
        ],
        parameters=[
            ParameterFile(
                param_file=LaunchConfiguration('params_file'),
                allow_substs=True,
            ),
            {
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool
                ),
                'image_topic': LaunchConfiguration('image_topic'),
                'publish_debug_image': ParameterValue(
                    LaunchConfiguration('publish_debug_image'),
                    value_type=bool,
                ),
                'model_path': LaunchConfiguration('model_path'),
            },
        ],
    )

    explorer_node = Node(
        package='fire_detection_drone',
        executable='drone_explorer',
        name='drone_explorer',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('start_explorer')),
        arguments=[
            '--ros-args',
            '--log-level',
            LaunchConfiguration('log_level'),
        ],
        parameters=[
            ParameterFile(
                param_file=LaunchConfiguration('params_file'),
                allow_substs=True,
            ),
            {
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool
                ),
                'pose_topic': LaunchConfiguration('pose_topic'),
                'pose_message_type': LaunchConfiguration('pose_message_type'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'exploration_pattern': LaunchConfiguration(
                    'exploration_pattern'
                ),
                'ned_input': ParameterValue(
                    LaunchConfiguration('ned_input'), value_type=bool
                ),
                'ned_output': ParameterValue(
                    LaunchConfiguration('ned_output'), value_type=bool
                ),
                'require_odometry': ParameterValue(
                    LaunchConfiguration('require_odometry'),
                    value_type=bool,
                ),
            },
        ],
    )

    return LaunchDescription(arguments + [detector_node, explorer_node])
