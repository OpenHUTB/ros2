#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CLI orchestration shared by ``python main.py`` and ``ros2 run``.

单行英文摘要 + 中文正文。
编排三个 case：跑规划 -> checker 校验 -> （可选）MuJoCo 闭环真飞第一个
架次 -> 结果写 results/case_<name>_result.json -> 终端摘要。
"""
from __future__ import annotations

import argparse
import json
import os
import time
from typing import List

from .core.cases import CASE_NAMES, get_case
from .core.checker import check_plan
from .core.energy import EnergyModel
from .core.planner import FleetPlan, RouteConfig, plan_routes

_RESULTS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results")


def run_case(name: str, sim: bool = True, progress: bool = False,
             cfg: RouteConfig | None = None) -> dict:
    """Plan + check (+ closed-loop fly) one case; returns the result dict."""
    cfg = cfg or RouteConfig()
    case = get_case(name)
    model = EnergyModel()
    t0 = time.time()
    plan = plan_routes(case, cfg, model)
    violations = check_plan(plan, case, model, cfg)
    result = dict(case=name,
                  outcome="ok" if not violations else "violation",
                  station_count=len(case.stations),
                  description=case.description,
                  plan=plan.to_dict(),
                  check=dict(violations=violations),
                  elapsed_s=round(time.time() - t0, 2))

    if sim:
        from .core.flyer import fly_sortie
        rep = fly_sortie(plan, 0, cfg, progress=progress)
        result["flight"] = rep.to_dict()

    os.makedirs(_RESULTS_DIR, exist_ok=True)
    path = os.path.join(_RESULTS_DIR, "%s_result.json" % name)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    print("[cli] results written to %s" % path)
    return result


def run_all(names: List[str], sim: bool = True, progress: bool = False) -> List[dict]:
    return [run_case(n, sim=sim, progress=progress) for n in names]


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="末端物流多站点配送与换电调度（MuJoCo 闭环验证）")
    p.add_argument("--case", default="all",
                   help="case 名（%s）或 all" % ", ".join(CASE_NAMES))
    p.add_argument("--no-sim", action="store_true", help="仅算法层（秒级，不需要 mujoco）")
    p.add_argument("--progress", action="store_true", help="打印仿真进度")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    sim = not args.no_sim
    names = list(CASE_NAMES) if args.case == "all" else [args.case]
    for n in names:
        if n not in CASE_NAMES:
            print("[cli] unknown case %r, expected one of %s" % (n, ", ".join(CASE_NAMES)))
            return 2
    results = run_all(names, sim=sim, progress=args.progress)

    print()
    print("=" * 84)
    print("%-18s %8s %8s %10s %12s %14s" %
          ("case", "sorties", "swaps", "duration_s", "distance_m", "arr_err_m"))
    for r in results:
        fl = r.get("flight")
        err = "%.2f" % fl["mean_arrival_error_m"] if fl else "-"
        print("%-18s %8d %8d %10.0f %12.0f %14s" %
              (r["case"], r["plan"]["sortie_count"], r["plan"]["swap_count"],
               r["plan"]["total_duration_s"], r["plan"]["total_distance_m"], err))
    print("[cli] violations: %s" %
          "; ".join("%s:%d" % (r["case"], len(r["check"]["violations"]))
                    for r in results))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
