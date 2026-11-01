#!/usr/bin/env python3
"""mujoco_ground_slam_nn_nav 统一入口。

支持：
    python3 main.py --launch teleop
    python3 main.py --launch slam_nav
    python3 main.py --launch neural_nav
    python3 main.py --launch e2e
    python3 main.py --test
    python3 main.py --info

ROS 节点通过 launch 启动；--test 只运行仓库内的单元测试，
不需要启动 roscore。
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import unittest

PACKAGE = "mujoco_ground_slam_nn_nav"
ROOT = os.path.dirname(os.path.realpath(__file__))
LAUNCHES = {
    "teleop": "teleop.launch",
    "slam_nav": "slam_nav.launch",
    "neural_nav": "neural_nav.launch",
    "e2e": "e2e.launch",
    "perception": "perception.launch",
    "benchmark": "benchmark.launch",
}


def print_info() -> int:
    print("package :", PACKAGE)
    print("root    :", ROOT)
    print("python  :", sys.version.split()[0])
    try:
        import mujoco
        print("mujoco  :", mujoco.__version__)
    except Exception as exc:
        print("mujoco  : NOT AVAILABLE (%s)" % exc)
    print("roslaunch:", shutil.which("roslaunch") or "NOT FOUND")
    print("venv    :", os.path.join(ROOT, ".venv"))
    return 0


def run_launch(name: str, scene: str, viewer: bool, extra: list[str]) -> int:
    if shutil.which("roslaunch") is None:
        print("roslaunch not found; source /opt/ros/noetic/setup.bash first", file=sys.stderr)
        return 1
    cmd = ["roslaunch", PACKAGE, LAUNCHES[name], "scene:=" + scene]
    if name in ("slam_nav", "neural_nav", "e2e", "perception", "teleop"):
        cmd.append("viewer:=" + ("true" if viewer else "false"))
    cmd.extend(extra)
    print("running:", " ".join(cmd), flush=True)
    return subprocess.call(cmd)


def run_tests() -> int:
    sys.path.insert(0, ROOT)
    suite = unittest.defaultTestLoader.discover(os.path.join(ROOT, "tests"))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1


def main() -> int:
    parser = argparse.ArgumentParser(description="MuJoCo 地面无人车 SLAM / 神经导航模块入口")
    parser.add_argument("--launch", choices=sorted(LAUNCHES), help="启动对应 ROS launch")
    parser.add_argument("--scene", default="warehouse",
                        choices=["warehouse", "obstacle_field", "crossing"])
    parser.add_argument("--no-viewer", action="store_true", help="不启动 OpenCV Viewer（无头运行）")
    parser.add_argument("--test", action="store_true", help="运行单元测试")
    parser.add_argument("--info", action="store_true", help="打印环境信息")
    parser.add_argument("args", nargs=argparse.REMAINDER,
                        help="追加给 roslaunch 的 key:=value 参数")
    ns = parser.parse_args()

    if ns.info:
        return print_info()
    if ns.test:
        return run_tests()
    if ns.launch:
        return run_launch(ns.launch, ns.scene, not ns.no_viewer, list(ns.args))
    parser.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
