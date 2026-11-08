#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""命令行入口（``ros2 run uav_right_of_way main`` 与 ``python main.py`` 共用）。"""
from __future__ import annotations

import argparse
import os
import sys

from .core.pipeline import (format_report, format_table, run_all, run_case,
                            to_json)
from .core.scenarios import SCENARIOS

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
_RESULTS_DIR = os.path.join(_PKG, "results")
_IMAGES_DIR = os.path.join(_PKG, "images")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="低空多机优先级通行权在线决策（基线 vs 规则启用，MuJoCo 闭环）")
    p.add_argument("--scenario", "--case", dest="scenario", default="all",
                   help="场景名（%s）或 all" % ", ".join(SCENARIOS))
    p.add_argument("--no-sim", action="store_true",
                   help="跳过 MuJoCo 闭环，仅算法层（秒级）")
    p.add_argument("--json", default="",
                   help="结果 JSON 路径（默认自动写 results/<场景>_result.json）")
    p.add_argument("--progress", action="store_true", help="打印仿真进度")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    sim = not args.no_sim

    if args.scenario == "all":
        results = run_all(list(SCENARIOS), sim=sim, progress=args.progress)
    else:
        if args.scenario not in SCENARIOS:
            print("未知场景 %r，可选：%s 或 all"
                  % (args.scenario, ", ".join(SCENARIOS)))
            return 2
        results = [run_case(args.scenario, sim=sim, progress=args.progress)]

    print()
    for r in results:
        print(format_report(r))
        print()

    # 每个场景自动落盘 results/<场景>_result.json（SPEC 要求）
    os.makedirs(_RESULTS_DIR, exist_ok=True)
    for r in results:
        p = os.path.join(_RESULTS_DIR, "%s_result.json" % r["scenario"])
        print("机读结果已写入 %s" % to_json([r], p))
    if args.json:
        print("机读结果已写入 %s" % to_json(results, args.json))

    print("轨迹俯视图/动图需要的话跑：python tools/make_gif.py")
    print(format_table(results))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
