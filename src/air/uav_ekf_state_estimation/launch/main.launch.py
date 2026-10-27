from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="uav_ekf_state_estimation",
                executable="ekf_state_estimation_node",
                name="uav_ekf_state_estimation",
                output="screen",
                parameters=[
                    {"gps_sigma_m": 1.8},
                    {"baro_sigma_m": 0.35},
                ],
            )
        ]
    )
