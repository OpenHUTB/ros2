import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("vehicle_wading_check"),
        "config",
        "params.yaml",
    )
    duration = LaunchConfiguration("duration")
    sim_node = Node(
        package="vehicle_wading_check",
        executable="main_sim",
        name="wading_sim",
        output="screen",
        parameters=[params_file, {"duration_s": duration}],
    )
    checker_node = Node(
        package="vehicle_wading_check",
        executable="main_check",
        name="wading_checker",
        output="screen",
        parameters=[params_file],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "duration",
            default_value="24.0",
            description="wading scenario duration in seconds",
        ),
        sim_node,
        checker_node,
        # The scenario node exits on its own once the profile is done; stopping
        # the whole launch keeps `ros2 launch` from hanging on the checker.
        RegisterEventHandler(
            OnProcessExit(
                target_action=sim_node,
                on_exit=[EmitEvent(event=Shutdown(reason="wading scenario finished"))],
            )
        ),
    ])
