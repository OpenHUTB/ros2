"""启动 qos_demo 的教学演示脚本（无需 rclpy 节点，直接跑 main.py）。

用法::

    ros2 launch qos_demo qos_demo.launch.py
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    script = PathJoinSubstitution(
        [FindPackageShare('qos_demo'), 'scripts', 'main.py'])
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', script, '--outdir', '/tmp/qos_demo'],
            output='screen'),
    ])
