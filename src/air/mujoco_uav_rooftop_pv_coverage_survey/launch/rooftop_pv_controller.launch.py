from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mujoco_uav_rooftop_pv_coverage_survey",
            executable="rooftop_pv_controller",
            name="rooftop_pv_controller",
            output="screen",
        )
    ])
