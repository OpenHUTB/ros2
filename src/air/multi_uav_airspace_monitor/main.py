#!/usr/bin/env python3
"""Entry point for the multi-UAV airspace monitor demo and offline checks."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from multi_uav_airspace_monitor.csv_export import write_separation_csv
from multi_uav_airspace_monitor.monitor import (
    evaluate_scenario,
    separation_rows_for_scenario,
)
from multi_uav_airspace_monitor.scenario import build_demo_scenario
from multi_uav_airspace_monitor.separation_plot import plot_separation_series
from multi_uav_airspace_monitor.visualize import render_scenario


def _write_extras(scenario: dict, csv_path: Path | None, plot_path: Path | None) -> None:
    """Write the optional separation CSV / chart from one shared set of rows."""
    if csv_path is None and plot_path is None:
        return
    rows = separation_rows_for_scenario(scenario)
    if csv_path is not None:
        written = write_separation_csv(rows, csv_path)
        print(f"    separation CSV: {written} ({len(rows)} rows)")
    if plot_path is not None:
        written_plot = plot_separation_series(rows, plot_path)
        print(f"    separation chart: {written_plot}")


def run_demo(output_dir: Path, csv_path: Path | None = None,
             plot_path: Path | None = None) -> int:
    """Render the built-in scenario; returns 0 whenever the demo completes."""
    output_dir.mkdir(parents=True, exist_ok=True)
    scenario = build_demo_scenario()
    summary = render_scenario(scenario, output_dir / "demo_screenshot.png")
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print("==> multi_uav_airspace_monitor")
    print(f"    status={summary['status']}")
    print(f"    uavs={summary['uav_count']} zones={summary['zone_count']}")
    print(f"    minimum_separation_m={summary['minimum_separation_m']}"
          f" (horizon {summary['horizon_s']} s)")
    print(f"    alerts={len(summary['alerts'])} conflicts={len(summary['conflicts'])}")
    for conflict in summary["conflicts"]:
        print(
            f"    conflict {conflict['uav_a']} vs {conflict['uav_b']}: "
            f"{conflict['separation_m']:.2f} m at {conflict['time_s']:.2f} s"
        )
    for alert in summary["alerts"]:
        print(f"    {alert['severity']}: {alert['message']}")
    print(f"    screenshot: {output_dir / 'demo_screenshot.png'}")
    _write_extras(scenario, csv_path, plot_path)
    return 0


def run_check(config_path: Path, csv_path: Path | None = None,
              plot_path: Path | None = None) -> int:
    """Evaluate a scenario JSON; returns 1 when the result is critical."""
    try:
        scenario = json.loads(config_path.read_text(encoding="utf-8"))
    except OSError as exc:
        print(f"cannot read {config_path}: {exc}", file=sys.stderr)
        return 2
    except json.JSONDecodeError as exc:
        print(f"{config_path} is not valid JSON: {exc}", file=sys.stderr)
        return 2
    try:
        summary = evaluate_scenario(scenario)
        if csv_path is not None or plot_path is not None:
            rows = separation_rows_for_scenario(scenario)
        else:
            rows = None
    except (KeyError, ValueError) as exc:
        print(f"invalid scenario {config_path}: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    if rows is not None:
        if csv_path is not None:
            written = write_separation_csv(rows, csv_path)
            print(f"separation CSV: {written} ({len(rows)} rows)", file=sys.stderr)
        if plot_path is not None:
            written_plot = plot_separation_series(rows, plot_path)
            print(f"separation chart: {written_plot}", file=sys.stderr)
    return 1 if summary["status"] == "critical" else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", nargs="?", default="demo", choices=["demo", "check"])
    parser.add_argument("--config", type=Path, help="scenario JSON for the check command")
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "results",
        help="directory for the demo screenshot and summary",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        help="also write the per-sample pairwise separation series to this CSV file",
    )
    parser.add_argument(
        "--plot",
        type=Path,
        help="also draw a separation-vs-time chart from the exported rows",
    )
    args = parser.parse_args(argv)
    if args.command == "check":
        if args.config is None:
            parser.error("check requires --config")
        return run_check(args.config, args.csv, args.plot)
    return run_demo(args.output, args.csv, args.plot)


if __name__ == "__main__":
    raise SystemExit(main())
