#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Command line orchestration shared by ``python main.py`` and
``ros2 run uav_rssi_source_locate main``.

Single-line summary: print the link budget table, run the analytic
trilateration sanity case, then (unless --no-sim) fly the whole
search/localize mission in MuJoCo and write results/<case>_result.json.
"""
from __future__ import annotations

import argparse
import json
import os

import numpy as np

from .core.cases import CASES, make_scenario
from .core.flyer import Beacon, locate_beacon
from .core.locator import (SearchCfg, best_waypoint, expanding_square,
                           localize, ring_points)
from .core.rssi import RssiCfg


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="UAV RSSI source localization: expanding-square search "
                    "+ RSSI trilateration (MuJoCo closed-loop verified)")
    p.add_argument("--case", default="all",
                   help="scenario name (%s) or all" % ", ".join(CASES))
    p.add_argument("--no-sim", action="store_true",
                   help="algorithm layer only, skip the MuJoCo flight")
    p.add_argument("--json", default="",
                   help="write combined result JSON to this path")
    p.add_argument("--progress", action="store_true",
                   help="print simulation progress")
    return p


def link_table(rssi_cfg: RssiCfg, alt_m: float) -> list:
    """链路面：距离 → 量化读数 → 反演距离，顺带给出单点测距误差。"""
    rows = []
    r_max = rssi_cfg.detect_range_m
    for d in (5.0, 10.0, 20.0, 30.0, r_max):
        r = rssi_cfg.rssi_at(d)
        d_inv = rssi_cfg.distance_from(r)
        rows.append(dict(distance_m=round(d, 2),
                         rssi_dbm=None if not np.isfinite(r) else round(r, 2),
                         inverted_m=round(d_inv, 2),
                         error_m=round(d_inv - d, 2)))
    return rows


def analytic_fix(scenario, cfg: SearchCfg, rssi_cfg: RssiCfg) -> dict:
    """算法层自检：真信标周围一圈干净样本，三边定位应回到它自己。

    只喂 (测量点三维位置, 量化 RSSI)，不给信标坐标；同时用同一批航点
    跑一次"取最强信号航点"的朴素基线，量出三边定位的收益。
    """
    bx, by = scenario.beacon_xy
    pts = ring_points((bx, by), cfg.fine_radius0_m, cfg.ring_points)
    samples = []
    for p in pts:
        p3 = np.array([p[0], p[1], cfg.alt_m])
        samples.append((p3, rssi_cfg.rssi_at(float(np.linalg.norm(
            p3 - np.array([bx, by, 0.5]))))))
    out = localize(samples, rssi_cfg)
    return dict(
        n_samples=out["n_samples"],
        feasible=out["feasible"],
        estimate_xy=[round(float(out["estimate"][0]), 2),
                     round(float(out["estimate"][1]), 2)]
        if out["feasible"] else None,
        error_m=round(float(np.linalg.norm(out["estimate"]
                                           - np.asarray([bx, by]))), 2)
        if out["feasible"] else None,
        residual_m=out["residual_m"])


def naive_grid_error(scenario, cfg: SearchCfg, rssi_cfg: RssiCfg) -> dict:
    """朴素基线：只按方形航点找信号最强的点，不做事后解算。"""
    bx, by = scenario.beacon_xy

    def rssi_fn(x, y):
        return rssi_cfg.rssi_at(float(np.linalg.norm(
            np.array([x, y, cfg.alt_m]) - np.array([bx, by, 0.5]))))

    wpts = expanding_square(cfg)
    est, peak = best_waypoint(wpts, rssi_fn)
    return dict(estimate_xy=[round(float(est[0]), 2), round(float(est[1]), 2)],
                peak_rssi_dbm=round(peak, 2),
                error_m=round(float(np.linalg.norm(est - np.asarray([bx, by]))),
                              2),
                n_waypoints=len(wpts))


def run_algorithm(scenario, cfg: SearchCfg, rssi_cfg: RssiCfg) -> dict:
    return dict(
        case=scenario.name,
        note=scenario.note,
        outcome="ok",
        algorithm=dict(
            area_m=list(cfg.area),
            alt_m=cfg.alt_m,
            grid_step_m=cfg.grid_step,
            detect_range_m=round(rssi_cfg.detect_range_m, 2),
            link_budget=link_table(rssi_cfg, cfg.alt_m),
            trilateration=analytic_fix(scenario, cfg, rssi_cfg),
            naive_best_waypoint=naive_grid_error(scenario, cfg, rssi_cfg)))


def run_closed_loop(scenario, cfg: SearchCfg, rssi_cfg: RssiCfg,
                    progress: bool = False) -> dict:
    res = locate_beacon(Beacon(scenario.beacon_xy), cfg, rssi_cfg,
                        progress=progress)
    print("[locator] %s: beacon %s, outcome %s, estimate %s, err %.2f m "
          "(residual %.2f m over %d samples, detected at %s s)"
          % (scenario.name, scenario.beacon_xy, res.outcome,
             "None" if res.estimate is None
             else "(%.1f, %.1f)" % (res.estimate[0], res.estimate[1]),
             res.error_m, res.residual_m, res.n_measurements,
             "%.1f" % res.detected_t if res.detected_t else "never"))
    print("           flight %.0f s simulated in %.1f s wall, "
          "verify %s" % (res.sim_s, res.wall_s, res.verified))
    out = res.as_dict()
    out["case"] = scenario.name
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
    rssi_cfg = RssiCfg()
    names = list(CASES) if args.case == "all" else [args.case]
    combined = []
    for name in names:
        sc = make_scenario(name)
        payload = run_algorithm(sc, sc.cfg, rssi_cfg)
        if not args.no_sim:
            payload["closed_loop"] = run_closed_loop(sc, sc.cfg, rssi_cfg,
                                                     args.progress)
        path = _write_result(name, payload)
        print("[locator] result written to %s" % path)
        combined.append(payload)
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(combined, f, ensure_ascii=False, indent=2)
        print("[locator] combined JSON -> %s" % args.json)
    return 0
