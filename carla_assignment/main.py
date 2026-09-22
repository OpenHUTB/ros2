#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA 无人车四次作业 · 一键汇总启动入口（main.py 约定）

作为整个 carla_assignment 的根入口：通过 --task 运行指定作业的 main.py。
满足老师"每个模块必须支持 main 脚本直接运行整个模块"的要求。

用法：
    python main.py --task control
    python main.py --task perception --waypoints "40,-8 40,12 25,20"
    python main.py --task navigation --goal 20,8
    python main.py --task end_to_end --mode test
"""

import argparse
import os
import subprocess
import sys

_ROOT = os.path.dirname(os.path.abspath(__file__))
TASKS = {
    "control":     ("01_control", "main.py"),
    "perception":  ("02_perception", "main.py"),
    "navigation":  ("03_navigation", "main.py"),
    "end_to_end":  ("04_end_to_end", "main.py"),
}


def main():
    p = argparse.ArgumentParser(description="Carla 无人车四次作业总入口")
    p.add_argument("--task", choices=list(TASKS.keys()), required=True,
                   help="要运行的作业")
    # 采用 parse_known_args 以便把 --xxx 选项透传（作业 --goal / --mode / --waypoints 等）
    args, unknown = p.parse_known_args()

    subdir, script = TASKS[args.task]
    script_path = os.path.join(_ROOT, subdir, script)
    cmd = [sys.executable, script_path] + unknown
    print(f"启动 {args.task}: {' '.join(cmd)}")
    return subprocess.call(cmd)


if __name__ == "__main__":
    sys.exit(main())
