# -*- coding: utf-8 -*-
"""ROS 2 launch 文件：启动 Dubins 曲线可视化节点。

无头机器（VM / CI）上交互模式会因没有显示设备而失败，因此把模式与输出
都暴露成 launch 参数：

    ros2 launch dubins_path main.launch.py                                    # 交互
    ros2 launch dubins_path main.launch.py mode:=record                       # 离屏录 gif
    ros2 launch dubins_path main.launch.py mode:=record output:=/tmp/dubins.gif
    ros2 launch dubins_path main.launch.py params:=/path/params.yaml
    ros2 launch dubins_path main.launch.py start:="150,480,0" end:="750,150,180" radius:=80
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    output = LaunchConfiguration('output')
    params = LaunchConfiguration('params')
    start = LaunchConfiguration('start')
    end = LaunchConfiguration('end')
    radius = LaunchConfiguration('radius')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='interactive',
            choices=['interactive', 'record'],
            description='interactive: 打开窗口实时演示；record: 离屏渲染并合成 gif'),
        DeclareLaunchArgument(
            'output', default_value='dubins_demo.gif',
            description='record 模式下的 gif 输出路径（相对节点工作目录）'),
        DeclareLaunchArgument(
            'params', default_value='',
            description='params.yaml 路径，留空则按安装空间/源码树自动查找'),
        DeclareLaunchArgument('start', default_value='',
                              description='起点位姿 x,y,yaw_deg，留空用 params.yaml/内置默认'),
        DeclareLaunchArgument('end', default_value='',
                              description='终点位姿 x,y,yaw_deg，留空用 params.yaml/内置默认'),
        DeclareLaunchArgument('radius', default_value='',
                              description='最小转弯半径（像素），留空用 params.yaml/内置默认'),
        Node(
            package='dubins_path',
            executable='main',
            name='dubins_visualizer',
            output='screen',
            arguments=['--mode', mode,
                       '--output', output,
                       '--params', params,
                       '--start', start,
                       '--end', end,
                       '--radius', radius],
        ),
    ])