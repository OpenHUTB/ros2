import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # 直接以脚本方式运行入口, 不依赖 colcon 安装
    script_dir = os.path.dirname(os.path.abspath(__file__))
    main_py = os.path.join(script_dir, "..", "main.py")

    return LaunchDescription([
        ExecuteProcess(
            cmd=["python3", main_py],
            output="screen",
        )
    ])
