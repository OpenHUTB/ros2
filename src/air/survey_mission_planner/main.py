#!/usr/bin/env python3
"""Entry point for the survey mission planner."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from survey_mission_planner.camera import CameraSpec
from survey_mission_planner.export import export_csv, export_kml, export_wpml
from survey_mission_planner.planner import SurveyPlanner
from survey_mission_planner.visualize import render_plan


def load_config(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def run_plan(config_path: Path, output_dir: Path) -> int:
    config = load_config(config_path)
    camera = CameraSpec(**config["camera"])
    planner = SurveyPlanner(
        camera=camera,
        target_gsd_m=float(config["target_gsd_m"]),
        forward_overlap=float(config.get("forward_overlap", 0.8)),
        side_overlap=float(config.get("side_overlap", 0.7)),
        speed_mps=float(config.get("speed_mps", 8.0)),
        turn_time_s=float(config.get("turn_time_s", 2.0)),
        max_flight_time_s=float(config.get("max_flight_time_s", 1500.0)),
        origin_lat=float(config.get("origin_lat", 28.22)),
        origin_lon=float(config.get("origin_lon", 112.94)),
    )
    polygon = np.asarray(config["polygon"], dtype=float)
    plan = planner.plan(polygon)
    output_dir.mkdir(parents=True, exist_ok=True)
    render_plan(polygon, plan, output_dir / "demo_screenshot.png")
    export_csv(plan, output_dir / "survey_waypoints.csv")
    export_kml(plan, output_dir / "survey_mission.kml")
    export_wpml(plan, output_dir / "survey_mission.wpml")
    (output_dir / "plan.json").write_text(
        json.dumps(plan.as_dict(), indent=2), encoding="utf-8"
    )
    print("==> survey_mission_planner")
    for key, value in plan.as_dict().items():
        if key not in {"warnings"}:
            print(f"    {key}={value}")
    for warning in plan.warnings:
        print(f"    warning: {warning}")
    print(f"    screenshot: {output_dir / 'demo_screenshot.png'}")
    return 0 if plan.coverage_ratio >= 0.90 else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", nargs="?", default="demo", choices=["demo", "plan", "test"])
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "demo_mission.json")
    parser.add_argument("--output", type=Path, default=ROOT / "results")
    args = parser.parse_args()
    if args.command == "test":
        import unittest

        suite = unittest.defaultTestLoader.discover(str(ROOT / "tests"))
        return 0 if unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful() else 1
    return run_plan(args.config, args.output)


if __name__ == "__main__":
    raise SystemExit(main())
