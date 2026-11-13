"""启动 kalman_filter_tracking 的教学演示脚本（无需 rclpy 节点，直接跑 main.py）。

用法::

    ros2 launch kalman_filter_tracking kalman_filter_tracking.launch.py
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    script = PathJoinSubstitution(
        [FindPackageShare('kalman_filter_tracking'), 'scripts', 'main.py'])
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', script, '--outdir', '/tmp/kalman_filter_tracking'],
            output='screen'),
    ])
