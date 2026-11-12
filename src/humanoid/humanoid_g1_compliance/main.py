#!/usr/bin/env python3
"""离线入口：不需要 ROS，也不需要显卡。

    python main.py --demo                 # 站立 + 一次手腕扰动，打印指标
    python main.py --demo --k-la 55       # 低刚度（让位更大）
    python main.py --bench                # 刚度扫描表
    python main.py --list                 # 列出刚度通道与取值范围
    python main.py --ros                  # 启动 ROS 2 仿真节点（需先 source）

等价命令：``ros2 run humanoid_g1_compliance demo``。
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from humanoid_g1_compliance.demo import list_stiffness, run_demo  # noqa: E402
from humanoid_g1_compliance.policy import DEFAULT_STIFFNESS_5     # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description='G1 全身柔顺控制（LAC）离线演示')
    ap.add_argument('--demo', action='store_true', help='站立 + 扰动演示')
    ap.add_argument('--bench', action='store_true', help='刚度扫描')
    ap.add_argument('--list', action='store_true', help='列出刚度通道')
    ap.add_argument('--ros', action='store_true', help='启动 ROS 2 仿真节点')
    ap.add_argument('--stiffness', type=float, nargs=5,
                    default=list(DEFAULT_STIFFNESS_5), help='5 通道刚度')
    ap.add_argument('--k-la', type=float, default=None, help='只改左臂线刚度')
    ap.add_argument('--force', type=float, default=6.0)
    ap.add_argument('--axis', default='y', choices=['x', 'y', 'z'])
    ap.add_argument('--body', default='left_wrist_roll_rubber_hand')
    ap.add_argument('--settle', type=float, default=3.0)
    ap.add_argument('--push-dur', type=float, default=2.0)
    ap.add_argument('--after', type=float, default=2.5)
    ap.add_argument('--device', default='cpu')
    args = ap.parse_args()

    if args.k_la is not None:
        args.stiffness = list(args.stiffness)
        args.stiffness[0] = args.k_la

    if args.list:
        return list_stiffness()
    if args.ros:
        from humanoid_g1_compliance.sim_node import main as ros_main
        ros_main()
        return 0
    if args.bench:
        here = os.path.dirname(os.path.abspath(__file__))
        return subprocess.call([sys.executable,
                                os.path.join(here, 'scripts', 'bench_compliance.py')])
    if args.demo:
        return run_demo(args)

    ap.print_help()
    return 0


if __name__ == '__main__':
    sys.exit(main())
