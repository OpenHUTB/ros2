#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
uav_lidar_dqn_navigation 统一入口

用法：
    python3 main.py --mode teleop       # 任务 1 键盘遥控
    python3 main.py --mode bridge       # 只启动 AirSim 桥接
    python3 main.py --mode scan         # 雷达扫描发布（建图用）
    python3 main.py --mode dqn          # 任务 2 DQN 导航策略部署
    python3 main.py --mode train        # 任务 2 DQN 训练
    python3 main.py --mode slam         # 任务 3 hector_slam 建图
    python3 main.py --mode camera       # 前置相机话题
    python3 main.py --mode e2e          # 任务 4 图像端到端 DQN 部署
    python3 main.py --mode e2e_train    # 任务 4 端到端 DQN 训练

launch 文件通过 args 传入 --mode，保证每个功能都能用 launch 一键启动。
"""

import argparse
import os
import sys

import rospy

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)


def run_bridge():
    from drone_bridge import DroneBridge
    DroneBridge().spin()


def run_teleop():
    from drone_bridge import DroneBridge
    from teleop_keyboard import KeyboardTeleop
    import threading

    bridge = DroneBridge()
    thread = threading.Thread(target=bridge.spin, name='drone_bridge')
    thread.daemon = True
    thread.start()
    KeyboardTeleop().run()


def run_scan():
    from lidar_scan_node import LidarScanNode
    LidarScanNode().run()


def run_dqn():
    from dqn_nav_node import DQNNavNode
    DQNNavNode().run()


def run_train():
    import train_dqn_nav
    sys.argv = ['train_dqn_nav.py'] + rospy.myargv()[2:]
    train_dqn_nav.main()


def run_slam():
    from lidar_scan_node import LidarScanNode
    LidarScanNode().run()


def run_camera():
    from camera_node import CameraNode
    CameraNode().run()


def run_e2e():
    from dqn_e2e_node import DQNE2ENode
    DQNE2ENode().run()


def run_e2e_train():
    import train_dqn_e2e
    sys.argv = ['train_dqn_e2e.py'] + rospy.myargv()[2:]
    train_dqn_e2e.main()


MODES = {
    'bridge': run_bridge,
    'teleop': run_teleop,
    'scan': run_scan,
    'slam': run_slam,
    'dqn': run_dqn,
    'train': run_train,
    'camera': run_camera,
    'e2e': run_e2e,
    'e2e_train': run_e2e_train,
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', required=True, choices=sorted(MODES.keys()))
    args = parser.parse_args()

    rospy.init_node('uav_lidar_dqn_' + args.mode)
    MODES[args.mode]()


if __name__ == '__main__':
    main()
