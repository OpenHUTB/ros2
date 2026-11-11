# -*- coding: utf-8 -*-
"""main.launch.py —— 一键启动「仿真 + 感知 + 控制」三节点闭环。

用法：
  ros2 launch uav_camera_tracking main.launch.py
  ros2 launch uav_camera_tracking main.launch.py scene:=occlusion
  ros2 launch uav_camera_tracking main.launch.py mode:=expert scene:=static
  ros2 launch uav_camera_tracking main.launch.py csv:=/tmp/uav_metrics.csv

可用 launch 参数：
  scene     static | moving | occlusion | gust        （默认 moving）
  mode      expert | two_stage | e2e                  （默认 two_stage）
  duration  总时长(s)，0 表示用场景自带时长            （默认 0）
  csv       回合指标追加写入的 CSV 路径，空表示不写     （默认 ''）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build(context):
    share = get_package_share_directory('uav_camera_tracking')
    params_file = os.path.join(share, 'config', 'params.yaml')

    scene = LaunchConfiguration('scene').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    csv_path = LaunchConfiguration('csv').perform(context)
    # launch 参数都是字符串，这里显式转成 double，否则节点声明 float 参数时类型不匹配
    duration = float(LaunchConfiguration('duration').perform(context) or 0.0)

    sim = Node(
        package='uav_camera_tracking', executable='sim', name='uav_sim',
        output='screen',
        parameters=[params_file, {
            'scene': scene, 'duration': duration, 'csv': csv_path,
        }],
    )
    detector = Node(
        package='uav_camera_tracking', executable='detector',
        name='uav_detector', output='screen',
        parameters=[params_file],
    )
    controller = Node(
        package='uav_camera_tracking', executable='controller',
        name='uav_controller', output='screen',
        parameters=[params_file, {'mode': mode}],
    )
    return [sim, detector, controller]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='moving',
                              description='测试场景：static/moving/occlusion/gust'),
        DeclareLaunchArgument('mode', default_value='two_stage',
                              description='控制器：expert/two_stage/e2e'),
        DeclareLaunchArgument('duration', default_value='0.0',
                              description='总时长(s)，0 表示用场景默认时长'),
        DeclareLaunchArgument('csv', default_value='',
                              description='指标 CSV 输出路径，空表示不写文件'),
        OpaqueFunction(function=_build),
    ])
