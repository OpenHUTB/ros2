"""启动人形机器人平衡行走控制节点（ROS 2 Humble）。

用法::

    ros2 launch humanoid_embodied_control main.launch.py
    ros2 launch humanoid_embodied_control main.launch.py goal_x:=3.0 goal_y:=0.5

参数：
  goal_x   目标点 x（米）
  goal_y   目标点 y（米）
  stride   单腿前后摆幅（弧度）
  lift     摆动腿额外屈膝量（弧度）
  period   一个完整步态周期的名义长度（秒）
  ctrl_hz  步态/平衡下发频率（低于 100 Hz 会摔倒，实测结论）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    stride = LaunchConfiguration('stride')
    lift = LaunchConfiguration('lift')
    period = LaunchConfiguration('period')
    ctrl_hz = LaunchConfiguration('ctrl_hz')

    walk_node = Node(
        package='humanoid_embodied_control',
        executable='humanoid_walk_node',
        name='humanoid_walk_node',
        output='screen',
        parameters=[{
            'goal_x': goal_x,
            'goal_y': goal_y,
            'stride': stride,
            'lift': lift,
            'period': period,
            'ctrl_hz': ctrl_hz,
        }],
        remappings=[
            ('/uav/lidar', '/scan'),
            ('/goal', '/move_base_simple/goal'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('goal_x', default_value='4.5', description='目标点 x（米）'),
        DeclareLaunchArgument('goal_y', default_value='0.0', description='目标点 y（米）'),
        DeclareLaunchArgument('stride', default_value='0.20', description='单腿前后摆幅（弧度）'),
        DeclareLaunchArgument('lift', default_value='0.10', description='摆动腿额外屈膝量（弧度）'),
        DeclareLaunchArgument('period', default_value='1.3', description='步态周期名义长度（秒）'),
        DeclareLaunchArgument('ctrl_hz', default_value='100.0',
                              description='步态/平衡下发频率（低于 100 会摔倒）'),
        walk_node,
    ])
