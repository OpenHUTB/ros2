import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("uav_wind_estimation"), "config", "demo.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("maneuver", default_value="circle"),
            Node(
                package="uav_wind_estimation",
                executable="wind_sim",
                name="wind_sim",
                output="screen",
                parameters=[
                    config,
                    {"maneuver": LaunchConfiguration("maneuver")},
                ],
            ),
            Node(
                package="uav_wind_estimation",
                executable="wind_estimator",
                name="wind_estimator",
                output="screen",
                parameters=[config],
            ),
        ]
    )
