#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
uav_nn_perception_navigation 统一入口

用法：
    python3 main.py --mode teleop        # 键盘遥控（任务 1）
    python3 main.py --mode bridge        # 只启动 AirSim 桥接节点
    python3 main.py --mode perception    # 激光雷达感知 + 神经网络控制（任务 2）
    python3 main.py --mode slam          # SLAM 建图（任务 3）
    python3 main.py --mode nav           # 基于地图的导航（任务 3）
    python3 main.py --mode end2end       # 图像到控制指令的端到端模型（任务 4）
    python3 main.py --mode train         # 端到端模型训练

也可以在 launch 文件里以 <node ... type="main.py"> 的方式调用，
此时用 args 传入 --mode。
"""

import argparse
import os
import sys
import threading

import rospy

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

MODES = ('bridge', 'teleop', 'perception', 'control', 'slam', 'nav', 'end2end', 'train')


def run_bridge():
    from airsim_bridge import AirSimBridgeNode
    node = AirSimBridgeNode()
    node.run()


def run_teleop():
    """桥接节点放后台线程，键盘节点占前台（键盘要读标准输入）。"""
    from airsim_bridge import AirSimBridgeNode
    from keyboard_teleop import KeyboardTeleop

    bridge = AirSimBridgeNode()
    t = threading.Thread(target=bridge.run, name='airsim_bridge')
    t.daemon = True
    t.start()

    KeyboardTeleop().run()


def run_perception():
    from lidar_perception import LidarPerceptionNode
    LidarPerceptionNode().run()


def run_control():
    from trajectory_control import TrajectoryControlNode
    TrajectoryControlNode().run()


def run_slam():
    from slam_node import SlamNode
    SlamNode().run()


def run_nav():
    from nav_node import NavNode
    NavNode().run()


def run_end2end():
    from end2end_node import End2EndNode
    End2EndNode().run()


def run_train():
    from train_end2end import main as train_main
    train_main()


DISPATCH = {
    'bridge': run_bridge,
    'teleop': run_teleop,
    'perception': run_perception,
    'control': run_control,
    'slam': run_slam,
    'nav': run_nav,
    'end2end': run_end2end,
    'train': run_train,
}


def main(argv=None):
    parser = argparse.ArgumentParser(description='UAV 神经网络感知导航模块统一入口')
    parser.add_argument('--mode', default='teleop', choices=MODES, help='运行模式')
    args, _unknown = parser.parse_known_args(argv)

    if args.mode != 'train':
        rospy.init_node('uav_' + args.mode)
    else:
        rospy.init_node('uav_train', disable_signals=True)

    try:
        DISPATCH[args.mode]()
    except ImportError as exc:
        rospy.logerr('模式 %s 尚未实现或依赖缺失: %s', args.mode, exc)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
