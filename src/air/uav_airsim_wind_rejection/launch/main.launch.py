"""launch 启动抗风补偿节点。"""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    share = get_package_share_directory("uav_airsim_wind_rejection")
    weights = os.path.join(share, "models_trained", "wind.npz")
    data = os.path.join(share, "models_trained", "wind_data.npz")

    return LaunchDescription([
        Node(
            package="uav_airsim_wind_rejection",
            executable="wind_rejection",
            name="uav_wind_rejection",
            output="screen",
            parameters=[{
                "weights": weights,
                "data": data,
                "cmd_scale": 3.0,
                "target_x": 0.0,
                "target_y": 0.0,
            }],
        ),
    ])
