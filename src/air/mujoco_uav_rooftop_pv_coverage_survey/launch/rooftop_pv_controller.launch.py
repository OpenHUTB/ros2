from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    config = Path(get_package_share_directory("mujoco_uav_rooftop_pv_coverage_survey")) / "config" / "controller.yaml"
    return LaunchDescription([
        Node(
            package="mujoco_uav_rooftop_pv_coverage_survey",
            executable="rooftop_pv_controller",
            name="rooftop_pv_controller",
            parameters=[str(config)],
            output="screen",
        )
    ])

