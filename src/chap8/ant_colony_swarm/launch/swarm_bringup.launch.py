from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    vision_node = Node(
        package="ant_colony_swarm",
        executable="vision_detect_node",
        output="screen"
    )
    ctrl_node = Node(
        package="ant_colony_swarm",
        executable="swarm_control_node",
        output="screen"
    )
    return LaunchDescription([
        vision_node,
        ctrl_node
    ])
