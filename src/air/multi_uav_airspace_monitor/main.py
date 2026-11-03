#!/usr/bin/env python3
"""Entry point for the multi-UAV airspace monitor demo and offline checks."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from multi_uav_airspace_monitor.monitor import evaluate_scenario
from multi_uav_airspace_monitor.scenario import build_demo_scenario
from multi_uav_airspace_monitor.visualize import render_scenario


def run_demo(output_dir: Path) -> int:
    output_dir.mkdir(parents=True, exist_ok=True)
    scenario = build_demo_scenario()
    summary = render_scenario(scenario, output_dir / "demo_screenshot.png")
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print("==> multi_uav_airspace_monitor")
    print(f"    status={summary['status']}")
    print(f"    uavs={summary['uav_count']} zones={summary['zone_count']}")
    print(f"    minimum_separation_m={summary['minimum_separation_m']}")
    print(f"    alerts={len(summary['alerts'])} conflicts={len(summary['conflicts'])}")
    for conflict in summary["conflicts"]:
        print(
            f"    conflict {conflict['uav_a']} vs {conflict['uav_b']}: "
            f"{conflict['separation_m']:.2f} m at {conflict['time_s']:.2f} s"
        )
    for alert in summary["alerts"]:
        print(f"    {alert['severity']}: {alert['message']}")
    print(f"    screenshot: {output_dir / 'demo_screenshot.png'}")
    return 0 if summary["conflicts"] and summary["alerts"] else 1


def run_check(config_path: Path) -> int:
    scenario = json.loads(config_path.read_text(encoding="utf-8"))
    summary = evaluate_scenario(scenario)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if summary["status"] in {"safe", "warning", "critical"} else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", nargs="?", default="demo", choices=["demo", "check"])
    parser.add_argument("--config", type=Path, help="scenario JSON for the check command")
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "results",
        help="directory for the demo screenshot and summary",
    )
    args = parser.parse_args()
    if args.command == "check":
        if args.config is None:
            parser.error("check requires --config")
        return run_check(args.config)
    return run_demo(args.output)


if __name__ == "__main__":
    raise SystemExit(main())
