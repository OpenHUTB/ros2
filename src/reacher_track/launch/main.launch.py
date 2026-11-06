#!/usr/bin/env python3
"""launch 启动 Reacher 机械臂目标跟踪的训练与评估流水线

用法（在 ROS2 workspace 源码树内启动）：
  cd ~/ros2_ws/src/reacher_track
  ros2 launch reacher_track main.launch.py

依次执行:
  1. collect_demo.py  : 用 IK+PD 专家生成监督演示数据
  2. train_mlp.py     : 监督训练 MLP 神经网络逼近专家控制律
  3. dagger_iter.py   : DAgger 迭代(用自身轨迹+专家标注)缓解分布偏移
  4. deploy_eval.py   : 闭环部署评估并输出误差曲线/性能表

动图渲染见 render_gif.py (python3 reacher_track/render_gif.py)。
"""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess

PKG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')


def generate_launch_description():
    pipeline = ("cd '%s' && python3 reacher_track/collect_demo.py && "
                "python3 reacher_track/train_mlp.py && "
                "python3 reacher_track/dagger_iter.py && "
                "python3 reacher_track/deploy_eval.py") % PKG_DIR
    return LaunchDescription([
        ExecuteProcess(cmd=['bash', '-c', pipeline], output='screen'),
    ])
