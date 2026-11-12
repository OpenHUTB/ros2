#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Command line orchestration shared by ``python main.py`` and
``ros2 run uav_moving_target_follow main``.

Single-line summary: run the algorithm layer (intercept plan + reference
audit) for each case, optionally the MuJoCo closed loop, print a
[follower]-style report and write results/<case>_result.json.
"""
from __future__ import annotations

import argparse
import json
import os

import numpy as np

from .core.follow import FollowCfg, follow_reference, plan_intercept
from .core.target import CASES, make_case


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="UAV standoff following of a moving ground target "
                    "(MuJoCo closed-loop verified)")
    p.add_argument("--case", default="all",
                   help="case name (%s) or all" % ", ".join(CASES))
    p.add_argument("--no-sim", action="store_true",
                   help="algorithm layer only, skip MuJoCo closed loop")
    p.add_argument("--json", default="",
                   help="write combined result JSON to this path")
    p.add_argument("--progress", action="store_true",
                   help="print simulation progress")
    return p


def run_algorithm(case, cfg: FollowCfg):
    """算法层：截获预测 + 沿全程采样核对 standoff 几何（纯解析）。"""
    plan = plan_intercept(case, (0.0, 0.0), cfg)
    samples = []
    prev = None
    for t in np.arange(0.0, case.duration_s + 1e-9, 1.0):
        p_t, v_t = case.pos_vel(float(t))
        p_ref, v_ref, d = follow_reference(p_t, v_t, cfg, prev)
        prev = d
        behind = float(np.dot(p_ref[:2] - p_t, d))
        samples.append(dict(
            t_s=round(float(t), 1),
            target_xy=[round(float(p_t[0]), 2), round(float(p_t[1]), 2)],
            ref_xy=[round(float(p_ref[0]), 2), round(float(p_ref[1]), 2)],
            behind_m=round(behind, 3),
            ref_alt_m=round(float(p_ref[2]), 3)))
    audited = all(abs(s["behind_m"] - cfg.standoff_m) < 1e-6 for s in samples)
    return {
        "case": case.name,
        "outcome": "ok",
        "algorithm": {
            "path_waypoints": [[float(x), float(y)] for x, y in case.waypoints],
            "target_speed_mps": case.speed,
            "path_length_m": round(case.total_len_m, 1),
            "duration_s": round(case.duration_s, 1),
            "standoff_m": cfg.standoff_m,
            "alt_m": cfg.alt_m,
            "intercept": plan,
            "reference_samples": samples[:8],
            "standoff_geometry_ok": audited,
        },
    }


def run_closed_loop(case, cfg: FollowCfg, progress: bool = False):
    from .core.flyer import follow_target, naive_above_target

    res = follow_target(case, cfg, progress=progress)
    naive = naive_above_target(case, cfg, res)
    print("[follower] %s: path %.0f m, target %.1f m/s, captured at %s s"
          % (case.name, case.total_len_m, case.speed,
             "%.2f" % res.intercept_s if res.intercept_s else "never"))
    print("           follow err mean %.2f / max %.2f m "
          "(turn max %.2f), naive-above turn max %.2f m, wall %.1f s"
          % (res.mean_err_m, res.max_err_m, res.turn_err_m, naive,
             res.wall_s))
    out = res.as_dict()
    out["naive_above_turn_max_m"] = round(naive, 2)
    out["case"] = case.name
    return out


def _write_result(case_name: str, payload: dict) -> str:
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out_dir = os.path.join(here, "results")
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "case_%s_result.json" % case_name)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
        f.write("\n")
    return path


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    cfg = FollowCfg()
    names = list(CASES) if args.case == "all" else [args.case]
    combined = []
    for name in names:
        case = make_case(name)
        payload = run_algorithm(case, cfg)
        if not args.no_sim:
            payload["closed_loop"] = run_closed_loop(case, cfg, args.progress)
        path = _write_result(name, payload)
        print("[follower] result written to %s" % path)
        combined.append(payload)
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(combined, f, ensure_ascii=False, indent=2)
        print("[follower] combined JSON -> %s" % args.json)
    return 0
