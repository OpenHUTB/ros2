#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""批量规划评估入口（骨架）。

下一步接入 moveit_commander：
1. 读取 targets.yaml 和 planners.yaml；
2. 对每个规划器、每个目标位姿重复规划；
3. 记录 success、planning_time、末端路径长度、位置/姿态误差；
4. 写出逐次记录 CSV 和汇总 CSV。

在实现完成前，本脚本只解析参数并打印配置，避免被误认为已经能跑实验。
"""

import argparse
import sys


def parse_args(argv):
    parser = argparse.ArgumentParser(description="MoveIt 六自由度机械臂规划器对比")
    parser.add_argument("--targets", required=True, help="目标位姿 YAML")
    parser.add_argument("--repeats", type=int, default=3, help="每个目标的重复次数")
    parser.add_argument("--timeout", type=float, default=5.0, help="单次规划超时（秒）")
    parser.add_argument("--out-dir", required=True, help="结果输出目录")
    return parser.parse_args(argv)


def main(argv):
    args = parse_args(argv)
    print("[moveit_arm_planning] plan_eval.py 尚未实现")
    print("[moveit_arm_planning] targets=%s" % args.targets)
    print("[moveit_arm_planning] repeats=%d" % args.repeats)
    print("[moveit_arm_planning] timeout=%.2f" % args.timeout)
    print("[moveit_arm_planning] out_dir=%s" % args.out_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
