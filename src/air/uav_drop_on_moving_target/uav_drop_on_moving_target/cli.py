#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""命令行入口：解算三个算例，可选跑 MuJoCo 闭环并写结果 JSON。"""
from __future__ import annotations

import argparse
import json
import os
import sys
from typing import List

from .core.cases import CASES, make_case
from .core.release import scatter_radius, solve_release

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
_RESULTS = os.path.join(_ROOT, "results")


def _fmt(v, nd=2):
    return "n/a" if v is None else ("%.*f" % (nd, v))


def run_case(name: str, simulate: bool) -> dict:
    case = make_case(name)
    sol = solve_release(case.track, case.vehicle, case.cfg)
    row = dict(case=case.name, title=case.title,
               vehicle_speed_mps=case.vehicle.speed,
               uav_speed_mps=case.track.speed,
               alt_m=case.cfg.alt_m,
               wind_mps=[round(float(w), 2) for w in case.cfg.wind],
               t_fall_s=round(sol.t_fall, 3),
               scatter_sigma_m=round(scatter_radius(case.cfg), 2),
               algorithm=sol.as_dict(),
               closed_loop=None)

    print("[solve] %-18s t_fall %.2f s | release at %s s | miss %s m | "
          "naive %s m | uniform %s m"
          % (case.name, sol.t_fall, _fmt(sol.t_release), _fmt(sol.miss_m),
             _fmt(sol.naive_miss_m), _fmt(sol.uniform_miss_m)))
    if not sol.feasible:
        print("        infeasible: %s" % sol.advice)
        return row

    if simulate:
        from .core.flyer import fly_case
        res = fly_case(case.track, case.vehicle, case.cfg)
        row["closed_loop"] = res.as_dict()
        rec = res.releases[0] if res.releases else None
        if rec is None:
            print("        MuJoCo: no release fired")
        else:
            print("        MuJoCo: release %.2f s at (%.1f, %.1f), impact error "
                  "%.2f m, track err %.2f m (near release %.2f m)"
                  % (rec.t_release, rec.release_pos[0], rec.release_pos[1],
                     rec.error_m, res.max_track_error_m,
                     res.max_error_release_m))
    return row


def main(argv: List[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="给行驶中的车队投送物资：投放时刻解算 + MuJoCo 闭环复核")
    ap.add_argument("--case", action="append", default=None,
                    help="算例名（可重复）；缺省跑全部")
    ap.add_argument("--list", action="store_true", help="只列出算例")
    ap.add_argument("--no-sim", action="store_true", help="只跑算法层，不进 MuJoCo")
    ap.add_argument("--json", default=None, help="把全部结果写到指定 JSON")
    args = ap.parse_args(argv)

    if args.list:
        for c in CASES:
            print("%-20s %s" % (c.name, c.title))
        return 0

    names = args.case or [c.name for c in CASES]
    rows = [run_case(n, simulate=not args.no_sim) for n in names]

    os.makedirs(_RESULTS, exist_ok=True)
    if not args.no_sim:
        for row in rows:
            if row["closed_loop"] is None:
                continue
            path = os.path.join(_RESULTS, "%s_result.json" % row["case"])
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(row, fh, ensure_ascii=False, indent=2)
            print("[result] wrote %s" % os.path.relpath(path, _ROOT))

    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(rows, fh, ensure_ascii=False, indent=2)
        print("[result] wrote %s" % args.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
