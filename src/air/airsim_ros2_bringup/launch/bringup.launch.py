import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_node(context):
    package_share = get_package_share_directory("airsim_ros2_bringup")
    return [
        Node(
            package="airsim_ros2_bringup",
            executable="main",
            name="airsim_ros2_bridge",
            output="screen",
            parameters=[
                os.path.join(package_share, "config", "bridge.yaml"),
                {
                    "host": LaunchConfiguration("host").perform(context),
                    "port": int(LaunchConfiguration("port").perform(context)),
                    "vehicle_name": LaunchConfiguration("vehicle_name").perform(
                        context
                    ),
                    "cmd_vel_frame": LaunchConfiguration("cmd_vel_frame").perform(
                        context
                    ),
                    "command_timeout": float(
                        LaunchConfiguration("command_timeout").perform(context)
                    ),
                    "use_sim_time": LaunchConfiguration("use_sim_time")
                    .perform(context)
                    .lower()
                    == "true",
                },
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("port", default_value="41451"),
            DeclareLaunchArgument("vehicle_name", default_value=""),
            DeclareLaunchArgument("cmd_vel_frame", default_value="body_flu"),
            DeclareLaunchArgument("command_timeout", default_value="0.5"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_launch_node),
        ]
    )
