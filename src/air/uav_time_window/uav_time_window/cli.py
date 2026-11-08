#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""命令行入口（``ros2 run uav_time_window main`` 与 ``python main.py`` 共用）。

运行结果自动落盘 ``results/<场景>_result.json``（逐场景）与
``results/all_results.json``（汇总），README 的数字与 JSON 同源。
"""
from __future__ import annotations

import argparse
import os

from .core.pipeline import format_report, run_all, run_case, to_json
from .core.scenarios import SCENARIOS


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="多机任务 4D 时间窗分配与延误均衡（MuJoCo 闭环验证）")
    p.add_argument("--scenario", "--case", dest="scenario", default="all",
                   help="场景名（%s）或 all" % ", ".join(SCENARIOS))
    p.add_argument("--no-sim", action="store_true", help="跳过 MuJoCo 闭环真飞")
    p.add_argument("--json", default="", help="额外把结果写入指定 JSON 文件")
    p.add_argument("--results-dir", default="results",
                   help="结果 JSON 落盘目录（默认 results/）")
    p.add_argument("--progress", action="store_true", help="打印仿真进度")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    if args.scenario == "all":
        names = list(SCENARIOS)
        results = run_all(names, sim=not args.no_sim, progress=args.progress)
    else:
        if args.scenario not in SCENARIOS:
            print("未知场景 %r，可选：%s" % (args.scenario, ", ".join(SCENARIOS)))
            return 2
        names = [args.scenario]
        results = [run_case(args.scenario, sim=not args.no_sim,
                            progress=args.progress)]

    print()
    for r in results:
        print(format_report(r))
        print()

    os.makedirs(args.results_dir, exist_ok=True)
    if len(results) == 1:
        out = os.path.join(args.results_dir, "%s_result.json" % results[0].name)
        print("机读结果已写入 %s" % to_json(results, out))
    else:
        for r in results:
            one = os.path.join(args.results_dir, "%s_result.json" % r.name)
            to_json([r], one)
        out = os.path.join(args.results_dir, "all_results.json")
        print("机读结果已写入 %s（逐场景 %s_result.json 一并落盘）"
              % (to_json(results, out), ", ".join(names)))
    if args.json:
        print("机读结果已写入 %s" % to_json(results, args.json))

    print("=" * 72)
    print("分配前后对比（贪心 EDF → 局部交换改进）")
    print("%-12s %12s %12s %14s %14s %10s" %
          ("场景", "总延误(前)", "总延误(后)", "最大单机(前)",
           "最大单机(后)", "目标(前→后)"))
    for r in results:
        g, f = r.greedy_report, r.final_report
        print("%-12s %12.1f %12.1f %14.1f %14.1f %6.1f→%.1f" %
              (r.name, g["delay_total"], f["delay_total"],
               g["delay_max_uav"], f["delay_max_uav"],
               g["objective"], f["objective"]))
    if results[0].sim is not None:
        print("闭环真飞：抽 %d 架/场景，实际到达与计划偏差、窗口状态见各 JSON sim 字段"
              % len(results[0].sim_uids))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
