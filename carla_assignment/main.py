#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA 作业一 · 一键启动入口（main.py 约定）

通过 --task 运行对应作业的 main.py。
本分支当前交付作业一（键盘运动控制）；
其余任务（感知/导航/端到端）后续以独立 PR 补充。

用法：
    python main.py --task control
    python main.py --task control --sim_time 20
"""

import argparse
import os
import subprocess
import sys

_ROOT = os.path.dirname(os.path.abspath(__file__))
TASKS = {
    "control": ("01_control", "main.py"),
}


def main():
    p = argparse.ArgumentParser(description="Carla 无人车作业一总入口")
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
