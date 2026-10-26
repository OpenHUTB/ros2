"""CARLA 作业综合整合与性能评价 —— ROS 2 Humble Launch 文件。

两种用法：

  1) 发布评测指标（默认）：读取已导出的 report.json，周期性发布到话题
         ros2 launch carla_benchmark_suite main.launch.py
         ros2 launch carla_benchmark_suite main.launch.py metrics_file:=/path/report.json

  2) 调度子模块：把 target 设成 control / perception / navigation / end_to_end，
     本 launch 会以独立进程拉起对应功能包的主入口
         ros2 launch carla_benchmark_suite main.launch.py target:=perception

说明：
    评测在**无 CARLA 服务端**的情况下也能完成（离线训练 + 合成环境回放）。
    若 target 为 control/perception/navigation/end_to_end 且需要连 CARLA，
    请把 host 指向宿主机 IP。CARLA 服务端启动、IP 与端口 2000 的查看等通用步骤见
    https://openhutb.github.io/ros2/set_up_and_connect_to_carla/ ，本模块不重复描述。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# target → 被调度功能包名
_TARGET_PKG = {
    'control': 'carla_keyboard_control',
    'perception': 'carla_perception_control',
    'navigation': 'carla_mapping_navigation',
    'end_to_end': 'carla_end_to_end_nn',
}


def _launch_setup(context, pkg_share):
    """按 target 决定：包含对应子功能包的 launch，还是启动指标发布节点。"""
    target = LaunchConfiguration('target').perform(context)
    host = LaunchConfiguration('host').perform(context)
    port = LaunchConfiguration('port').perform(context)
    metrics_file = LaunchConfiguration('metrics_file').perform(context)

    # ---- 调度模式：包含子功能包自己的 launch 文件 ----
    # 直接复用子包的 launch，避免在本包内重复描述其参数与节点定义。
    if target in _TARGET_PKG:
        pkg = _TARGET_PKG[target]
        sibling = os.path.join(
            get_package_share_directory(pkg), 'launch', 'main.launch.py')
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sibling),
            launch_arguments={
                'host': host,
                'port': port,
                'goal': LaunchConfiguration('goal').perform(context),
                'model_path': LaunchConfiguration('model_path').perform(context),
            }.items(),
        )]

    # ---- 默认：启动指标发布节点 ----
    return [Node(
        package='carla_benchmark_suite',
        executable='benchmark_node',
        name='carla_benchmark_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'sim_params.yaml'), {
            'target': target,
            'metrics_file': metrics_file,
        }],
    )]


def generate_launch_description():
    pkg_share = get_package_share_directory('carla_benchmark_suite')

    return LaunchDescription([
        DeclareLaunchArgument(
            'target', default_value='',
            description='调度哪个子模块：control / perception / navigation / end_to_end；'
                        '留空则本节点只发布评测指标'),
        DeclareLaunchArgument(
            'host', default_value='127.0.0.1',
            description='CARLA 服务端地址（虚拟机运行客户端时填宿主机 IP）'),
        DeclareLaunchArgument('port', default_value='2000', description='CARLA RPC 端口'),
        DeclareLaunchArgument(
            'metrics_file', default_value='',
            description='已导出的评测结果 JSON（main.py --benchmark --out report.json）'),
        DeclareLaunchArgument('goal', default_value='20,8',
                              description='透传给子模块的导航目标（供 navigation 使用）'),
        DeclareLaunchArgument('model_path', default_value='models/nn_plan.json',
                              description='透传给子模块的模型路径'),
        OpaqueFunction(function=_launch_setup, args=[pkg_share]),
    ])
