from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="two_link_arm_ik",
             executable="two_link_arm_ik",
             name="two_link_arm_ik"),
    ])
