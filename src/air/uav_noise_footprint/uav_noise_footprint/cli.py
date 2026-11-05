#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""命令行入口（``ros2 run uav_noise_footprint main`` 与 ``python main.py`` 共用）。"""
from __future__ import annotations

import argparse
import os

from .core.pipeline import format_report, run_all, run_case, to_json
from .core.scenarios import SCENARIOS

_HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS_DIR = os.path.join(_HERE, "results")
IMAGES_DIR = os.path.join(_HERE, "images")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="城市低空噪声足迹评估与航线降噪（MuJoCo 闭环验证）")
    p.add_argument("--scenario", default="all",
                   help="场景名（%s）或 all" % ", ".join(SCENARIOS))
    p.add_argument("--case", default=None,
                   help="同 --scenario 的别名")
    p.add_argument("--no-sim", action="store_true", help="跳过 MuJoCo 闭环复飞")
    p.add_argument("--json", default="", help="把结果写入 JSON 文件")
    p.add_argument("--no-figures", action="store_true", help="跳过图生成")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    scenario = args.case or args.scenario

    if scenario == "all":
        results = run_all(list(SCENARIOS), sim=not args.no_sim)
    else:
        if scenario not in SCENARIOS:
            print("未知场景 %r，可选：%s" % (scenario, ", ".join(SCENARIOS)))
            return 2
        results = [run_case(scenario, sim=not args.no_sim)]

    print()
    for r in results:
        print(format_report(r))
        print()

    # 结果落盘（README 的数字必须与这里的 JSON 一致）
    os.makedirs(RESULTS_DIR, exist_ok=True)
    per_case = []
    for r in results:
        path = os.path.join(RESULTS_DIR, "%s_result.json" % r.name)
        per_case.append(to_json([r], path))
        print("机读结果已写入 %s" % path)
    all_path = os.path.join(RESULTS_DIR, "all_result.json")
    to_json(results, all_path)
    print("机读结果已写入 %s" % all_path)
    if args.json:
        print("机读结果已写入 %s" % to_json(results, args.json))

    if not args.no_figures:
        from .figures import draw_footprint_all
        os.makedirs(IMAGES_DIR, exist_ok=True)
        print("足迹等值线图已写入 %s" %
              draw_footprint_all(results, IMAGES_DIR))

    # 汇总表
    print("=" * 72)
    print("%-12s %12s %12s %9s %10s" %
          ("场景", "目标(dB)前", "目标(dB)后", "航程增幅", "闭环误差"))
    for r in results:
        s = r.summary
        cl = s.get("closed_loop")
        err = "—" if cl is None else "%.2f m" % max(
            cl["before"]["sim"]["max_track_error"],
            cl["after"]["sim"]["max_track_error"])
        print("%-12s %12.1f %12.1f %8.2f%% %10s" %
              (r.name, s["objective_before"], s["objective_after"],
               s["path_increase_pct"], err))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
