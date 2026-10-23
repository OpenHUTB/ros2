from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="multi_uav_airspace_monitor",
                executable="airspace_monitor_node",
                name="multi_uav_airspace_monitor",
                output="screen",
                parameters=[{"zones_json": "[]"}],
            )
        ]
    )
