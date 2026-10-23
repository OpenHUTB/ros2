#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓取任务封装（骨架）。

本脚本只负责调用 marm_planning 里已有的 moveit_pick_and_place_demo.py
并记录耗时，不重复实现抓取算法。
"""

import argparse
import subprocess
import sys
import time


def parse_args(argv):
    parser = argparse.ArgumentParser(description="MoveIt 抓取任务封装")
    parser.add_argument(
        "--demo",
        default="moveit_pick_and_place_demo.py",
        help="marm_planning/scripts 下的抓取 demo 文件名",
    )
    return parser.parse_args(argv)


def main(argv):
    args = parse_args(argv)
    print("[moveit_arm_planning] pick_place_task.py 尚未实现")
    print("[moveit_arm_planning] 计划调用 marm_planning/%s" % args.demo)
    start = time.time()
    print("[moveit_arm_planning] 当前为骨架，耗时 %.3f s" % (time.time() - start))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
