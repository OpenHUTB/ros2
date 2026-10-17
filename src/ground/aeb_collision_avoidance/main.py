#!/usr/bin/env python3
"""Course entry point: run the standalone AEB simulation, or the ROS 2 node with --ros."""

from __future__ import annotations

import argparse
from pathlib import Path

from aeb_collision_avoidance.aeb_core import AEBConfig  # noqa: F401  (re-exported for convenience)
from aeb_collision_avoidance.simulator import run_simulation, save_report


def run_demo(output_dir: str) -> int:
    """Run the dependency-free simulation and write the plot plus CSV report."""
    result = run_simulation()
    plot_path, report_path = save_report(result, output_dir)
    finite_ttcs = [value for value in result.ttcs if value != float("inf")]
    print("AEB simulation completed")
    print(f"  steps: {len(result.times)}")
    print(f"  final distance: {result.final_distance:.2f} m")
    print(f"  final speed: {result.final_speed:.2f} m/s")
    print(f"  stopped: {'yes' if result.stopped else 'no'}")
    print(f"  collision: {'yes' if result.collision else 'no'}")
    print(f"  largest finite TTC: {max(finite_ttcs, default=0.0):.2f} s")
    print(f"  plot: {plot_path}")
    print(f"  data: {report_path}")
    return 0 if not result.collision else 1


def main() -> int:
    parser = argparse.ArgumentParser(description="TTC-based autonomous emergency braking demo")
    parser.add_argument("--ros", action="store_true", help="run the ROS 2 node instead of the standalone demo")
    parser.add_argument("--output-dir", default="results", help="directory for the plot and CSV report")
    args = parser.parse_args()
    if args.ros:
        from aeb_collision_avoidance.aeb_node import main as ros_main

        return ros_main()
    return run_demo(str(Path(args.output_dir)))


if __name__ == "__main__":
    raise SystemExit(main())
