"""统一入口：ros2 run uav_slam_nav main 或 python main.py。"""
import sys

from launch import LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def main():
    share = get_package_share_directory("uav_slam_nav")
    launch_file = os.path.join(share, "launch", "main.launch.py")
    ls = LaunchService()
    ls.include_launch_description(
        IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file)).perform(None)
    )
    return ls.run()


if __name__ == "__main__":
    sys.exit(main())
