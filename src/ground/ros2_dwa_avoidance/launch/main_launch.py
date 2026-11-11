import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("ros2_dwa_avoidance")
    params_file = os.path.join(share, "config", "dwa_params.yaml")

    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    max_v = LaunchConfiguration("max_v")

    return LaunchDescription([
        DeclareLaunchArgument("goal_x", default_value="8.0",
                              description="goal x in the odom frame [m]"),
        DeclareLaunchArgument("goal_y", default_value="0.0",
                              description="goal y in the odom frame [m]"),
        DeclareLaunchArgument("max_v", default_value="0.8",
                              description="maximum linear speed [m/s]"),
        Node(
            package="ros2_dwa_avoidance",
            executable="main_dwa",
            name="dwa_avoidance",
            output="screen",
            parameters=[
                params_file,
                {
                    "goal_x": ParameterValue(goal_x, value_type=float),
                    "goal_y": ParameterValue(goal_y, value_type=float),
                    "max_v": ParameterValue(max_v, value_type=float),
                },
            ],
        ),
    ])
