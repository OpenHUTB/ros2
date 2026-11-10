"""统一入口：ros2 run ugv_slam_nav main 或 python main.py。"""
import os
import sys
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("ugv_slam_nav")
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "main.launch.py"))
        )
    ])


def main():
    from launch import LaunchService
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    return ls.run()


if __name__ == "__main__":
    sys.exit(main())
