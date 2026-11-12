#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DAVE 水下仿真启动器：把命令行参数转换为 run_demo.sh 的 [mode] [headless] 参数。

同时兼容 `ros2 launch` 传入的 `--ros-args` / remap 参数（这些参数会被忽略）。
"""
import os
import sys

MODES = ("demo", "valve", "plug")


def _share_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("dave_underwater_sim")
    except Exception:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def demo_args(argv):
    """从 argv 中提取 run_demo.sh 需要的参数，忽略 ROS 参数/remap（含 '--ros-args'、'-r'、'__node:=...'）。"""
    mode = "demo"
    headless = False
    for a in argv:
        if a.startswith("-") or "=" in a:
            continue
        if a in MODES:
            mode = a
        elif a == "headless":
            headless = True
    out = [mode]
    if headless:
        out.append("headless")
    return out


def main(argv=None):
    extra = demo_args(sys.argv[1:] if argv is None else argv)
    script = os.path.join(_share_dir(), "run_demo.sh")
    if not os.path.isfile(script):
        raise SystemExit("missing run_demo.sh: " + script)
    os.execv("/bin/bash", ["bash", script] + extra)


if __name__ == "__main__":
    main()
