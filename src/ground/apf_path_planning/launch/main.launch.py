# -*- coding: utf-8 -*-
"""ROS 2 launch 文件：启动 APF 路径规划可视化节点。

无头机器（VM / CI）上交互模式会因没有显示设备而失败，所以把模式与输出
暴露成 launch 参数：

    ros2 launch apf_path_planning main.launch.py                              # 交互
    ros2 launch apf_path_planning main.launch.py mode:=record                 # 离屏录 gif
    ros2 launch apf_path_planning main.launch.py mode:=record output:=/tmp/apf_demo.gif
    ros2 launch apf_path_planning main.launch.py params:=/path/params.yaml
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
            'output', default_value='apf_demo.gif',
            description='record 模式下的 gif 输出路径（相对节点工作目录）'),
        DeclareLaunchArgument(
            'params', default_value='',
            description='params.yaml 路径，留空则按安装空间/源码树自动查找'),
        Node(
            package='apf_path_planning',
            executable='main',
            name='apf_visualizer',
            output='screen',
            arguments=['--mode', mode,
                       '--output', output,
                       '--params', params],
        ),
    ])