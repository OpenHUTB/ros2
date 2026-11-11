#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Command-line orchestration shared by `python main.py` and `ros2 run`."""
from __future__ import annotations

import argparse
import json
import os
import time
from dataclasses import asdict
from typing import List, Optional

from .core.checker import TAG as CHECKER_TAG, assess
from .core.flyer import TAG as FLYER_TAG, FlightConfig, fly_first_lines
from .core.planner import TAG as PLANNER_TAG, SprayConfig, plan_field
from .core.scenarios import CASES, FieldCase, case_outline, get_case
from .core.sweep import MIN_COVERAGE, TAG as SWEEP_TAG, recommend, sweep_case
from . import viz

RESULT_NAME = "uav_agri_spray_result.json"


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Agricultural UAV spray coverage planning and "
                    "deposition assessment (MuJoCo closed loop)")
    p.add_argument("--case", default="all",
                   help="case name (%s) or all" % ", ".join(c.name for c in CASES))
    p.add_argument("--no-sim", action="store_true",
                   help="algorithm layer only, skip the MuJoCo closed loop")
    p.add_argument("--json", default="",
                   help="write machine-readable results here "
                        "(default: results/%s)" % RESULT_NAME)
    p.add_argument("--progress", action="store_true", help="sim progress print")
    return p


def run_case(case: FieldCase, base_cfg: SprayConfig) -> dict:
    """Plan → assess → sweep for one field case; returns JSON-ready dict."""
    print("%s case=%s: min-width sweep, spacing=%.2f m" %
          (PLANNER_TAG, case.name, base_cfg.spacing))
    path = plan_field(case.subregions, case.obstacles, base_cfg)
    print("%s lines=%d length=%.1f m duration=%.1f s" %
          (PLANNER_TAG, len(path.lines), path.total_length, path.duration))

    outline = case_outline(case)
    _grid, met = assess(path, outline, base_cfg)
    print("%s coverage=%.1f%% overlap=%.1f%% missed=%.1f m2" %
          (CHECKER_TAG, 100 * met.coverage_ratio, 100 * met.overlap_ratio,
           met.missed_area_m2))

    points = sweep_case(case, base_cfg)
    rec = recommend(points)
    print("%s recommended speed=%.1f m/s overlap=%.0f%% "
          "coverage=%.1f%% duration=%.1f s" %
          (SWEEP_TAG, rec.speed, 100 * rec.overlap, 100 * rec.coverage_ratio,
           rec.duration_s))

    return {
        "case": case.name,
        "field_area_m2": met.field_area_m2,
        "planner": {
            "num_scanlines": len(path.lines),
            "subregions": path.subregions,
            "spacing_m": round(base_cfg.spacing, 2),
            "total_length_m": round(path.total_length, 1),
            "duration_s": round(path.duration, 1),
            "speed_mps": base_cfg.speed,
        },
        "deposition": {
            "coverage_ratio": met.coverage_ratio,
            "overlap_ratio": met.overlap_ratio,
            "missed_area_m2": met.missed_area_m2,
            "mean_deposit": met.mean_deposit,
            "threshold": met.threshold,
        },
        "sweep": {
            "speeds": [3.0, 4.0, 5.0],
            "overlaps": [0.1, 0.2, 0.3],
            "min_coverage": MIN_COVERAGE,
            "points": [asdict(p) for p in points],
            "recommended": {
                "speed_mps": rec.speed,
                "overlap": rec.overlap,
                "coverage_ratio": rec.coverage_ratio,
                "duration_s": rec.duration_s,
            },
        },
        "coverage_map": viz.map_payload(_grid, path.points),
    }


def run_closed_loop(case: FieldCase, cfg: SprayConfig, n_lines: int = 6,
                    progress: bool = False) -> dict:
    """Fly the first n_lines of the recommended plan, re-assess deposition."""
    path = plan_field(case.subregions, case.obstacles, cfg)
    print("%s flying first %d lines of %s (speed=%.1f overlap=%.0f%%) "
          "in MuJoCo..." % (FLYER_TAG, n_lines, case.name, cfg.speed,
                            100 * cfg.overlap))
    res = fly_first_lines(path, n_lines, cfg, case_outline(case),
                          FlightConfig(), progress=progress)
    print("%s wall=%.1f s max_track_err=%.2f m final_err=%.2f m" %
          (FLYER_TAG, res.wall_time_s, res.max_track_error_m,
           res.final_error_m))
    print("%s planned coverage=%.1f%% -> actual coverage=%.1f%% "
          "(delta %+.1f%%)" %
          (FLYER_TAG, 100 * res.planned.coverage_ratio,
           100 * res.actual.coverage_ratio,
           100 * (res.actual.coverage_ratio - res.planned.coverage_ratio)))
    return {
        "case": case.name,
        "lines_flown": res.flown_lines,
        "speed_mps": cfg.speed,
        "overlap": cfg.overlap,
        "flown_length_m": res.flown_length_m,
        "sim": {
            "steps": res.steps,
            "wall_time_s": res.wall_time_s,
            "max_track_error_m": res.max_track_error_m,
            "final_error_m": res.final_error_m,
        },
        "planned_coverage_ratio": res.planned.coverage_ratio,
        "actual_coverage_ratio": res.actual.coverage_ratio,
        "delta_coverage": round(res.actual.coverage_ratio
                                - res.planned.coverage_ratio, 4),
        "actual_missed_area_m2": res.actual.missed_area_m2,
        "actual_overlap_ratio": res.actual.overlap_ratio,
        "threshold": res.actual.threshold,
        "actual_map": res.actual_map,
    }


def default_json_path() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, "..", "results", RESULT_NAME))


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    t0 = time.time()
    base_cfg = SprayConfig()

    cases = CASES if args.case == "all" else [get_case(args.case)]
    missions = [run_case(c, base_cfg) for c in cases]

    closed_loop: Optional[dict] = None
    if not args.no_sim:
        rec = missions[0]["sweep"]["recommended"]
        cfg = SprayConfig(**{**base_cfg.__dict__,
                             "overlap": rec["overlap"],
                             "speed": rec["speed_mps"]})
        closed_loop = run_closed_loop(cases[0], cfg, n_lines=6,
                                      progress=args.progress)
    else:
        print("%s --no-sim: closed loop skipped" % FLYER_TAG)

    report = {
        "outcome": "ok",
        "tool": "uav_agri_spray",
        "missions": missions,
        "closed_loop": closed_loop,
    }

    json_path = args.json or default_json_path()
    os.makedirs(os.path.dirname(json_path), exist_ok=True)
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=1)
    print("%s results written to %s" % (SWEEP_TAG, json_path))

    out_dir = os.path.normpath(os.path.join(
        os.path.dirname(json_path), "..", "images"))
    viz.write_maps(report, out_dir)

    print()
    for m in missions:
        p, d, r = m["planner"], m["deposition"], m["sweep"]["recommended"]
        print("%-22s lines=%2d len=%7.1f m  coverage=%5.1f%%  "
              "rec %0.1fm/s@%d%% -> %.1f s" %
              (m["case"], p["num_scanlines"], p["total_length_m"],
               100 * d["coverage_ratio"], r["speed_mps"],
               round(100 * r["overlap"]), r["duration_s"]))
    if closed_loop:
        print("closed-loop(%s): planned %.1f%% -> actual %.1f%%, "
              "wall %.1f s" %
              (closed_loop["case"], 100 * closed_loop["planned_coverage_ratio"],
               100 * closed_loop["actual_coverage_ratio"],
               closed_loop["sim"]["wall_time_s"]))
    print("total wall %.1f s" % (time.time() - t0))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
