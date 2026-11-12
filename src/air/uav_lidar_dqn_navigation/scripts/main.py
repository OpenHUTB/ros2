#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
uav_lidar_dqn_navigation 统一入口

用法：
    python3 main.py --mode teleop       # 任务 1 键盘遥控
    python3 main.py --mode bridge       # 只启动 AirSim 桥接
    python3 main.py --mode scan         # 雷达扫描发布（建图感知层）
    python3 main.py --mode train        # 任务 2 DQN 策略训练
    python3 main.py --mode dqn          # 任务 2 DQN 导航部署
    python3 main.py --mode camera       # 前置相机话题
    python3 main.py --mode e2e          # 任务 4 图像端到端 DQN 部署
    python3 main.py --mode e2e_train    # 任务 4 端到端 DQN 训练

建图（任务 3）需要 launch 组合桥接、扫描、gmapping 与导航多个节点，
因此没有单独的模式，统一用 main.launch mode:=slam。
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


def run_train():
    import train_dqn_nav
    # myargv: [脚本, '--mode', 'train', 其余训练参数...]
    sys.argv = ['train_dqn_nav.py'] + rospy.myargv()[3:]
    train_dqn_nav.main()


def run_dqn():
    from dqn_nav_node import DQNNavNode
    DQNNavNode().run()


def run_camera():
    from camera_node import CameraNode
    CameraNode().run()


def run_e2e():
    from dqn_e2e_node import DQNE2ENode
    DQNE2ENode().run()


def run_e2e_train():
    import train_dqn_e2e
    # myargv: [脚本, '--mode', 'e2e_train', 其余训练参数...]
    sys.argv = ['train_dqn_e2e.py'] + rospy.myargv()[3:]
    train_dqn_e2e.main()


MODES = {
    'bridge': run_bridge,
    'teleop': run_teleop,
    'scan': run_scan,
    'train': run_train,
    'dqn': run_dqn,
    'camera': run_camera,
    'e2e': run_e2e,
    'e2e_train': run_e2e_train,
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', required=True, choices=sorted(MODES.keys()))
    # 训练模式要把 --ip/--episodes 等参数透传给 train_dqn_nav.py，
    # 顶层只能用 parse_known_args；其它模式仍对未知参数报错。
    args, unknown = parser.parse_known_args()
    if unknown and args.mode not in ('train', 'e2e_train'):
        parser.error('unrecognized arguments: %s' % ' '.join(unknown))

    rospy.init_node('uav_lidar_dqn_' + args.mode)
    MODES[args.mode]()


if __name__ == '__main__':
    main()
