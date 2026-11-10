import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("uav_control_authority_guard"), "config", "demo.yaml"
    )
    simulator = Node(
        package="uav_control_authority_guard",
        executable="control_authority_sim",
        name="uav_control_authority_sim",
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
        package="uav_control_authority_guard",
        executable="control_authority_guard",
        name="uav_control_authority_guard",
        output="screen",
        parameters=[config],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("duration", default_value="0.0"),
            simulator,
            guard,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=simulator,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="control authority scenario reached its duration"
                            )
                        )
                    ],
                )
            ),
        ]
    )
