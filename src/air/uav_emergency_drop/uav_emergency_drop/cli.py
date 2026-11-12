#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Command line orchestration shared by ``python main.py`` and
``ros2 run uav_emergency_drop main``.

Single-line summary: run algorithm layer (select/plan/ballistics table) for
each case, optionally the MuJoCo closed loop, print [planner]-style report
and write results/<case>_result.json with English snake_case keys.
"""
from __future__ import annotations

import argparse
import json
import os
from typing import List

import numpy as np

from .core.ballistics import conditions_table
from .core.cases import CASES
from .core.planner import CargoCfg, plan_missions


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="UAV emergency supply drop: point selection & release "
                    "timing (MuJoCo closed-loop verified)")
    p.add_argument("--case", default="all",
                   help="case name (%s) or all" % ", ".join(CASES))
    p.add_argument("--no-sim", action="store_true",
                   help="algorithm layer only, skip MuJoCo closed loop")
    p.add_argument("--json", default="",
                   help="write combined result JSON to this path")
    p.add_argument("--progress", action="store_true",
                   help="print simulation progress")
    return p


def run_algorithm(case):
    """算法层：选点 + 架次 + 弹道表 + 航线预检（纯解析，秒级）。"""
    selected, missions = plan_missions(case.points, case.cargo)
    total_w = sum(p.weight for p in case.points)
    covered_w = sum(p.weight for p in case.points if p.pid in selected)
    served = sum(p.demand for p in case.points if p.pid in selected)
    skipped = [p.pid for p in case.points if p.pid not in selected]

    # 航线预检：沿闭环航线逐段检查提前量可行性（河流 case 含桥梁绕行）
    from .core.trigger import check_leg
    from .core.flyer import build_route
    wps, _tt, _idx, _nc = build_route(case)
    leg_checks = []
    if case.river is not None:
        from .core.planner import segment_crosses_band
        direct = [case.cargo.depot, tuple(wps[3])]
        leg_checks.append(dict(
            leg="depot->first target direct",
            crosses_river=bool(segment_crosses_band(direct[0], direct[1],
                                                    *case.river)),
            detour_inserted=True))
    for i in range(1, len(wps)):
        tgt = np.asarray(wps[i], dtype=float)
        if i - 1 < len(_tt) and i in _idx:
            tgt = _tt[_idx.index(i)]
        win = check_leg(wps[i - 1], wps[i], tgt, case.plan)
        leg_checks.append(dict(leg="(%d,%d)->(%d,%d)"
                               % (wps[i - 1][0], wps[i - 1][1],
                                  wps[i][0], wps[i][1]),
                               **win.as_dict()))

    return {
        "case": case.name,
        "outcome": "ok",
        "algorithm": {
            "demand_points": [
                dict(pid=p.pid, x=p.x, y=p.y, demand=p.demand,
                     urgency=p.urgency, weight=p.weight)
                for p in case.points],
            "selected": selected,
            "skipped": skipped,
            "covered_weight": covered_w,
            "total_weight": total_w,
            "units_served": served,
            "missions": [m.as_dict() for m in missions],
            "ballistics_table": conditions_table(case.ballistics),
            "release_plan": case.plan.as_dict(),
            "leg_checks": leg_checks,
        },
        "closed_loop": {"simulated": False},
        "_case_obj": case,
    }


def run_case(name: str, sim: bool = True, progress: bool = False) -> dict:
    case = CASES[name]
    result = run_algorithm(case)
    if not sim:
        print("[planner] %s: selected %d/%d points, covered weight %.0f/%.0f"
              % (name, len(result["algorithm"]["selected"]),
                 len(case.points), result["algorithm"]["covered_weight"],
                 result["algorithm"]["total_weight"]))
        return result

    from .core.flyer import build_route, fly_drop_route
    route, true_targets, target_idx, no_cut = build_route(case)
    print("[trigger] %s: route %s" % (name,
          [[round(x), round(y)] for x, y in route]))
    fly = fly_drop_route(route, true_targets, case.plan,
                         target_idx=target_idx, progress=progress,
                         no_cut=no_cut)
    print("[flyer] %s: %d releases, impact err mean %.2f m, "
          "track err max %.2f m, wall %.1f s"
          % (name, len(fly.releases),
             fly.as_dict()["impact_error_mean_m"],
             fly.max_track_error_m, fly.wall_s))
    result["closed_loop"] = fly.as_dict()
    return result


def write_results(results: List[dict], path: str) -> str:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    out = []
    for r in results:
        r = dict(r)
        r.pop("_case_obj", None)
        out.append(r)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(out if len(out) > 1 else out[0], f,
                  ensure_ascii=False, indent=2)
    return path


def write_case_results(results: List[dict], out_dir: str) -> List[str]:
    """每个 case 单独落盘 results/<case>_result.json（可复现的机读结果）。"""
    paths = []
    for r in results:
        path = os.path.join(out_dir, "%s_result.json" % r["case"])
        write_results([r], path)
        paths.append(path)
    return paths


def format_report(r: dict) -> str:
    a = r["algorithm"]
    lines = ["[planner] case %s (%s)" % (r["case"], r["outcome"]),
             "  selected: %s  skipped: %s" % (a["selected"], a["skipped"]),
             "  covered weight: %.0f / %.0f, units served: %d"
             % (a["covered_weight"], a["total_weight"], a["units_served"])]
    for m in a["missions"]:
        lines.append("  sortie %d: %s load=%d dist=%.0fm flight=%.0fs "
                     "depart=%.0fs"
                     % (m["sortie"], m["pids"], m["load"], m["distance_m"],
                        m["flight_s"], m["depart_s"]))
    rp = a["release_plan"]
    lines.append("  release plan: h=%.0fm v=%.0fm/s t_fall=%.2fs "
                 "lead=%.1fm sigma=%.1fm drift=(%.1f, %.1f)m"
                 % (rp["h_m"], rp["speed_mps"], rp["t_fall_s"], rp["lead_m"],
                    rp["sigma_m"], rp["drift_m"][0], rp["drift_m"][1]))
    c = r["closed_loop"]
    if c.get("simulated"):
        lines.append("  closed loop: %d releases, impact err mean %.2f / "
                     "max %.2f m, track err max %.2f m, wall %.1f s"
                     % (len(c["releases"]), c["impact_error_mean_m"],
                        c["impact_error_max_m"], c["max_track_error_m"],
                        c["wall_time_s"]))
    else:
        lines.append("  closed loop: skipped (--no-sim)")
    return "\n".join(lines)


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.case == "all":
        names = list(CASES)
    else:
        if args.case not in CASES:
            print("unknown case %r, choose from: %s"
                  % (args.case, ", ".join(CASES)))
            return 2
        names = [args.case]

    results = [run_case(n, sim=not args.no_sim, progress=args.progress)
               for n in names]

    print()
    for r in results:
        print(format_report(r))
        print()

    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out_dir = os.path.join(here, "results")
    paths = write_case_results(results, out_dir)
    for p in paths:
        print("[planner] result written to %s" % p)
    if args.json:
        print("[planner] combined result written to %s"
              % write_results(results, args.json))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
