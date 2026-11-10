"""ugv_slam_nav 一键启动：odom_tf + simple_nav + nav_relay + slam_toolbox + nav2。"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("ugv_slam_nav")
    slam_params = os.path.join(share, "config", "slam_toolbox.yaml")
    nav2_params = os.path.join(share, "config", "nav2_params.yaml")
    return LaunchDescription([
        Node(package="ugv_slam_nav", executable="ugv_odom_tf",
             name="ugv_odom_tf", output="screen"),
        Node(package="ugv_slam_nav", executable="ugv_simple_nav",
             name="ugv_simple_nav", output="screen"),
        Node(package="ugv_slam_nav", executable="ugv_nav_relay",
             name="ugv_nav_relay", output="screen",
             parameters=[{"nav2_timeout": 1.5, "max_vx": 0.5, "max_yaw": 1.0}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("slam_toolbox"),
                             "launch", "online_async_launch.py")),
            launch_arguments={"slam_params_file": slam_params}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("nav2_bringup"),
                             "launch", "bringup_launch.py")),
            launch_arguments={"params_file": nav2_params,
                              "use_sim_time": "True"}.items(),
        ),
    ])
