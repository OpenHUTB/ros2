#!/usr/bin/env python3
"""Entry point for the EKF demo, evaluation and tests."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from uav_ekf_state_estimation.simulation import run_simulation
from uav_ekf_state_estimation.visualize import render_result


def run_demo(output_dir: Path, duration: float, seed: int) -> int:
    output_dir.mkdir(parents=True, exist_ok=True)
    result = run_simulation(duration_s=duration, seed=seed)
    render_result(result, output_dir / "demo_screenshot.png")
    (output_dir / "metrics.json").write_text(
        json.dumps(result.as_dict(), indent=2), encoding="utf-8"
    )
    print("==> uav_ekf_state_estimation")
    for key, value in result.metrics.items():
        print(f"    {key}={value}")
    print(f"    screenshot: {output_dir / 'demo_screenshot.png'}")
    return 0 if result.metrics["ekf_rmse_m"] < result.metrics["gps_rmse_m"] else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", nargs="?", default="demo", choices=["demo", "test"])
    parser.add_argument("--output", type=Path, default=ROOT / "results")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()
    if args.command == "test":
        import unittest

        suite = unittest.defaultTestLoader.discover(str(ROOT / "tests"))
        return 0 if unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful() else 1
    return run_demo(args.output, args.duration, args.seed)


if __name__ == "__main__":
    raise SystemExit(main())
