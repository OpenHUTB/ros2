import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("uav_airsim_imu_velocity_estimator")
    weights = os.path.join(share, "models_trained", "imu_vel.npz")

    estimator = Node(
        package="uav_airsim_imu_velocity_estimator",
        executable="imu_velocity_estimator",
        name="uav_imu_velocity_estimator",
        output="screen",
        parameters=[{"weights": weights, "vel_scale": 4.0}],
        remappings=[("/uav/imu", "/uav/imu"), ("/uav/velocity_est", "/uav/velocity_est")],
    )

    return LaunchDescription([estimator])
