from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="nn_avoidance",
            executable="infer_node",
            output="screen"
        )
    ])