#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""命令行入口（``ros2 run uav_deconflict main`` 与 ``python main.py`` 共用）。"""
from __future__ import annotations

import argparse

from .core.pipeline import format_report, run_all, run_case, to_json
from .core.scenarios import SCENARIOS


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="低空多机 4D 航迹冲突检测与解脱（MuJoCo 闭环验证）")
    p.add_argument("--scenario", default="all",
                   help="场景名（%s）或 all" % ", ".join(SCENARIOS))
    p.add_argument("--no-sim", action="store_true", help="跳过 MuJoCo 闭环复飞")
    p.add_argument("--json", default="", help="把结果写入 JSON 文件")
    p.add_argument("--progress", action="store_true", help="打印仿真进度")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    if args.scenario == "all":
        results = run_all(list(SCENARIOS), sim=not args.no_sim,
                          progress=args.progress)
    else:
        if args.scenario not in SCENARIOS:
            print("未知场景 %r，可选：%s" % (args.scenario, ", ".join(SCENARIOS)))
            return 2
        results = [run_case(args.scenario, sim=not args.no_sim,
                            progress=args.progress)]

    print()
    for r in results:
        print(format_report(r))
        print()

    if args.json:
        print("机读结果已写入 %s" % to_json(results, args.json))

    print("=" * 72)
    print("%-10s %10s %10s %10s %14s" %
          ("场景", "初始冲突", "消解后", "总延误s", "真飞最小间隔m"))
    for r in results:
        s = r.summary
        truth = s.get("truth_after", {}).get("min_h")
        print("%-10s %10d %10d %10.1f %14s" %
              (r.name, s["before"]["conflicts"], s["after"]["conflicts"],
               s["delay_total"], "%.1f" % truth if truth is not None else "—"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
