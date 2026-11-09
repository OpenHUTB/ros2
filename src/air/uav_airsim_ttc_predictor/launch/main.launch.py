"""launch 启动 TTC 预测节点。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("uav_airsim_ttc_predictor")
    weights = os.path.join(share, "models_trained", "ttc.npz")

    return LaunchDescription([
        Node(
            package="uav_airsim_ttc_predictor",
            executable="ttc_predictor",
            name="uav_ttc_predictor",
            output="screen",
            parameters=[{"weights": weights, "ttc_max": 20.0}],
        ),
    ])