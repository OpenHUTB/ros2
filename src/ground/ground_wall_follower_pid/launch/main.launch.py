from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ground_wall_follower_pid",
            executable="wall_scan_sim",
            name="wall_scan_sim",
            output="screen",
            parameters=[{"use_sim_time": False}],
        ),
        Node(
            package="ground_wall_follower_pid",
            executable="wall_follower",
            name="wall_follower",
            output="screen",
            parameters=[{"use_sim_time": False}],
        ),
    ])
