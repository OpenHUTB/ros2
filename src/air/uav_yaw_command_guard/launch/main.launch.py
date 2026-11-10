import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("uav_yaw_command_guard"),
        "config",
        "params.yaml",
    )
    simulator = Node(
        package="uav_yaw_command_guard",
        executable="yaw_demo_sim",
        name="uav_yaw_demo_sim",
        output="screen",
        parameters=[
            config,
            {
                "duration_s": ParameterValue(
                    LaunchConfiguration("duration"), value_type=float
                )
            },
        ],
    )
    guard = Node(
        package="uav_yaw_command_guard",
        executable="yaw_command_guard",
        name="uav_yaw_command_guard",
        output="screen",
        parameters=[config],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("duration", default_value="8.0"),
            guard,
            simulator,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=simulator,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="yaw demo simulator reached duration"
                            )
                        )
                    ],
                )
            ),
        ]
    )
