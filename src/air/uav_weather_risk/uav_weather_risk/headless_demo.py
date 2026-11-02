"""Offline demo entry point, usable without a ROS 2 installation."""

from __future__ import annotations

import argparse
import csv
import json
import os

from .core.scenario import run_demo
from .plotting import animate_demo, plot_risk, plot_sources, plot_trajectory


def write_csv(records, path: str) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fields = list(records[0].keys())
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(records)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Run the deterministic weather-risk demo")
    parser.add_argument("--duration", type=float, default=60.0, help="simulation seconds")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output-dir", default="artifacts/demo")
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument("--no-gif", action="store_true")
    args = parser.parse_args(argv)

    result = run_demo(duration_s=args.duration, seed=args.seed)
    os.makedirs(args.output_dir, exist_ok=True)
    write_csv(list(result.records), os.path.join(args.output_dir, "timeseries.csv"))
    with open(os.path.join(args.output_dir, "summary.json"), "w", encoding="utf-8") as stream:
        json.dump(result.summary, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    if not args.no_plots:
        plot_trajectory(result.records, os.path.join(args.output_dir, "trajectory.png"))
        plot_risk(result.records, os.path.join(args.output_dir, "risk_timeline.png"))
        plot_sources(result.records, os.path.join(args.output_dir, "wind_sources.png"))
    if not args.no_plots and not args.no_gif:
        animate_demo(result.records, os.path.join(args.output_dir, "demo.gif"))
    print(json.dumps(result.summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
