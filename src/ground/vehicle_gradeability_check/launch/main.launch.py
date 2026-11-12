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
        get_package_share_directory("vehicle_gradeability_check"),
        "config",
        "params.yaml",
    )
    simulator = Node(
        package="vehicle_gradeability_check",
        executable="main_sim",
        name="vehicle_gradeability_demo_sim",
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
    checker = Node(
        package="vehicle_gradeability_check",
        executable="main_check",
        name="vehicle_gradeability_check",
        output="screen",
        parameters=[config],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("duration", default_value="10.0"),
            checker,
            simulator,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=simulator,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="gradeability demo simulator reached duration"
                            )
                        )
                    ],
                )
            ),
        ]
    )
