#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
uav_lidar_dqn_navigation 统一入口

用法：
    python3 main.py --mode teleop       # 任务 1 键盘遥控
    python3 main.py --mode bridge       # 只启动 AirSim 桥接
    python3 main.py --mode scan         # 雷达扫描发布（建图感知层）

本阶段实现桥接、遥控与雷达感知；DQN 训练/部署、建图与端到端控制
在后续 PR 中按层加入，并同步扩充这里的模式表。
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


MODES = {
    'bridge': run_bridge,
    'teleop': run_teleop,
    'scan': run_scan,
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', required=True, choices=sorted(MODES.keys()))
    args = parser.parse_args()

    rospy.init_node('uav_lidar_dqn_' + args.mode)
    MODES[args.mode]()


if __name__ == '__main__':
    main()
