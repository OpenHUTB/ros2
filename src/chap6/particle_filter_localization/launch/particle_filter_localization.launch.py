"""启动 particle_filter_localization 的教学演示脚本（无需 rclpy 节点，直接跑 main.py）。

用法::

    ros2 launch particle_filter_localization particle_filter_localization.launch.py
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    script = PathJoinSubstitution(
        [FindPackageShare('particle_filter_localization'), 'scripts', 'main.py'])
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', script, '--outdir', '/tmp/particle_filter_localization'],
            output='screen'),
    ])
