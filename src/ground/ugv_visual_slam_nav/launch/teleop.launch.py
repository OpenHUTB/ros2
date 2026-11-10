"""键盘遥控：只起仿真节点，再配合 teleop_twist_keyboard 手动开车。

    ros2 launch ugv_visual_slam_nav teleop.launch.py
    # 另开一个终端：
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/ugv/cmd_vel

这一项对应作业里"通过键盘对机器人进行运动控制"。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scene = LaunchConfiguration('scene')
    seed = LaunchConfiguration('seed')

    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='corridor',
                              description='corridor | cluttered | dynamic | night'),
        DeclareLaunchArgument('seed', default_value='0'),
        LogInfo(msg='键盘遥控：另开终端运行 '
                    'ros2 run teleop_twist_keyboard teleop_twist_keyboard '
                    '--ros-args -r cmd_vel:=/ugv/cmd_vel'),
        Node(package='ugv_visual_slam_nav', executable='sim',
             name='ugv_sim', output='screen',
             parameters=[{'scene': scene, 'seed': seed, 'rate_hz': 10.0}]),
    ])
