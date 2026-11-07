"""ROS 2 启动文件：以进程方式拉起车载端到端仿真。

该模块本质是 Pygame 独立仿真（需要图形界面），因此用 ExecuteProcess 直接运行
已安装的控制台脚本 vehicle_sim，而非作为 ROS 节点注册。

说明：默认运行方式为 `python3 main.py` 或 `bash main.sh`；本 launch 用于在
ROS 2 工作空间（colcon build + source）内的统一拉起。
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['vehicle_sim'],
            output='screen',
        )
    ])
