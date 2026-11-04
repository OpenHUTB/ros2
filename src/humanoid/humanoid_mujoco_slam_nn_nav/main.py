#!/usr/bin/env python3
"""人形机器人模块统一入口。

    python3 main.py --mode test            # 单元测试（不需要 ROS）
    python3 main.py --mode train           # 重新训练四个网络
    python3 main.py --mode eval            # 三场景评测
    python3 main.py --mode teleop --scene warehouse
    python3 main.py --mode nav    --scene corridor
    python3 main.py --mode e2e    --scene corridor

teleop/nav/e2e 会转调 roslaunch；其余直接运行仓库内脚本。
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys

PKG = os.path.dirname(os.path.abspath(__file__))
PKG_NAME = "humanoid_mujoco_slam_nn_nav"


def run_py(script, extra):
    cmd = [sys.executable, os.path.join(PKG, script)] + extra
    return subprocess.call(cmd, cwd=PKG)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="nav",
                    choices=["test", "train", "eval", "teleop", "nav", "e2e", "benchmark"])
    ap.add_argument("--scene", default="warehouse")
    ap.add_argument("--seconds", default=None)
    args, extra = ap.parse_known_args()

    if args.mode == "test":
        return run_py(os.path.join("tests", "run_all.py"), [])
    if args.mode == "train":
        return run_py(os.path.join("tools", "train_nets.py"), extra)
    if args.mode == "eval":
        return run_py(os.path.join("tools", "evaluate.py"), extra)

    launch = {"teleop": "teleop.launch", "nav": "slam_nav.launch",
              "e2e": "e2e.launch", "benchmark": "benchmark.launch"}[args.mode]
    cmd = ["roslaunch", PKG_NAME, launch, f"scene:={args.scene}"] + extra
    return subprocess.call(cmd)


if __name__ == "__main__":
    sys.exit(main())
