from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pedestrian_group_crossing",
            executable="group_crossing_node",
            name="pedestrian_group_crossing",
            output="screen",
        )
    ])
