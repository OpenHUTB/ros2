# -*- coding: utf-8 -*-
"""ROS 2 launch 文件：启动 Bezier 曲线可视化节点。

无头机器（VM / CI）上默认的交互模式会因为没有显示设备而失败，
所以这里把「模式」和「输出」都暴露成 launch 参数：

    ros2 launch bezier_trajectory main.launch.py                        # 交互
    ros2 launch bezier_trajectory main.launch.py mode:=record           # 离屏录 gif
    ros2 launch bezier_trajectory main.launch.py mode:=record output:=/tmp/b.gif
    ros2 launch bezier_trajectory main.launch.py params:=/path/params.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    output = LaunchConfiguration('output')
    params = LaunchConfiguration('params')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='interactive',
            choices=['interactive', 'record'],
            description='interactive: 打开窗口实时演示；record: 离屏渲染并合成 gif'),
        DeclareLaunchArgument(
            'output', default_value='bezier_demo.gif',
            description='record 模式下的 gif 输出路径（相对节点工作目录）'),
        DeclareLaunchArgument(
            'params', default_value='',
            description='params.yaml 路径，留空则按安装空间/源码树自动查找'),
        Node(
            package='bezier_trajectory',
            executable='main',
            name='bezier_visualizer',
            output='screen',
            arguments=['--mode', mode,
                       '--output', output,
                       '--params', params],
        ),
    ])