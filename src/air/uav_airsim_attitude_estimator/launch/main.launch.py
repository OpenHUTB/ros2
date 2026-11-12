"""launch 入口：启动 IMU 姿态估计节点。"""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("uav_airsim_attitude_estimator")
    weights = os.path.join(share, "models_trained", "attitude.npz")
    return LaunchDescription([
        Node(
            package="uav_airsim_attitude_estimator",
            executable="attitude_estimator",
            name="uav_airsim_attitude_estimator",
            output="screen",
            parameters=[{"weights": weights}],
        )
    ])