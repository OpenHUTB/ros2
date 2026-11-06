from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    """启动一次真实扫掠：等价于 python main.py sim。"""
    return LaunchDescription([
        ExecuteProcess(
            cmd=["python3", "main.py", "sim"],
            output="screen",
        )
    ])
