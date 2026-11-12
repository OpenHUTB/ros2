#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""命令行入口（``ros2 run uav_vertiport_queue main`` 与 ``python main.py`` 共用）。"""
from __future__ import annotations

import argparse
import os

from .core.capacity import DEFAULT_WAIT_THRESHOLD
from .core.figures import capacity_figure, wait_figure
from .core.pipeline import format_report, run_all, run_capacity, to_json
from .core.scenarios import SCENARIOS

# 模块目录：cli.py 位于 <模块>/uav_vertiport_queue/cli.py，向上两级
_HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS_DIR = os.path.join(_HERE, "results")
IMAGES_DIR = os.path.join(_HERE, "images")
DEFAULT_JSON = os.path.join(RESULTS_DIR, "vertiport_queue_result.json")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="低空起降场进离场排队与容量评估（MuJoCo 闭环验证）")
    p.add_argument("--scenario", default="all",
                   help="场景名（%s）或 all" % ", ".join(SCENARIOS))
    p.add_argument("--no-sim", action="store_true",
                   help="跳过 MuJoCo 闭环抽飞（仅算法层，秒级）")
    p.add_argument("--json", default=DEFAULT_JSON,
                   help="结果 JSON 落盘路径")
    p.add_argument("--progress", action="store_true", help="打印仿真进度")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    names = list(SCENARIOS) if args.scenario == "all" else [args.scenario]
    for n in names:
        if n not in SCENARIOS:
            print("未知场景 %r，可选：%s 或 all" % (n, ", ".join(SCENARIOS)))
            return 2

    sim = not args.no_sim
    if sim:
        print("运行场景与 MuJoCo 闭环抽飞（--no-sim 可跳过）...")
    results = run_all(names, sim=sim, progress=args.progress)
    print()
    for r in results:
        print(format_report(r))
        print()

    print("容量评估：固定请求率扫描 ...")
    capacity = run_capacity()
    scan = capacity["scan"]
    sat = capacity["saturation"]
    print("%-10s %10s %10s %10s %10s" %
          ("请求率", "吞吐量", "平均等待", "最大队列", "积压"))
    for row in scan:
        print("%-10s %10.1f %10.1f %10d %10d" %
              (row["rate"], row["throughput"], row["avg_wait"],
               row["max_queue_length"], row["backlog"]))
    print("饱和点：请求率 %.0f 架次/h（平均等待 %.1f s）" %
          (sat["rate"], sat["avg_wait"]))

    cap_path = capacity_figure(scan, sat,
                               os.path.join(IMAGES_DIR, "capacity_curve.png"),
                               DEFAULT_WAIT_THRESHOLD)
    wait_path = wait_figure(results, os.path.join(IMAGES_DIR,
                                                  "wait_comparison.png"))
    print("已生成 %s" % cap_path)
    print("已生成 %s" % wait_path)

    if args.json:
        os.makedirs(os.path.dirname(args.json) or ".", exist_ok=True)
        print("机读结果已写入 %s" % to_json(results, capacity, args.json))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
