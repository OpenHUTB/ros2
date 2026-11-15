#!/usr/bin/env python3
"""CLI entry point for the randomized DWA stress benchmark."""

from __future__ import annotations

import argparse
import os

if __package__:
    from .benchmark import run_benchmark
else:  # direct execution from the package root
    from benchmark import run_benchmark


def main() -> None:
    parser = argparse.ArgumentParser(description="Random-scene DWA stress benchmark")
    parser.add_argument("--out", default="benchmark_results.json")
    parser.add_argument("--figdir", default="figures")
    parser.add_argument("--densities", default="4,7,10,13",
                        help="comma separated obstacle counts per world")
    parser.add_argument("--worlds", type=int, default=10,
                        help="worlds per density")
    parser.add_argument("--no-figures", action="store_true",
                        help="skip matplotlib plots (minimal ROS environments)")
    parser.add_argument("--edge-threshold", type=float, default=0.31,
                        help="clearance threshold (m) for the edge-ratio metric")
    args = parser.parse_args()

    densities = tuple(int(v) for v in args.densities.split(",") if v.strip())
    figdir = "" if args.no_figures else args.figdir
    payload = run_benchmark(args.out, figdir, densities, args.worlds,
                            edge_threshold=args.edge_threshold)

    print("\n=== summary ===")
    print(f"{'density':>7} {'controller':>10} {'runs':>4} {'success':>8} "
          f"{'time*':>7} {'clear!':>7} {'acc!':>7} {'jerk!':>7} {'edge%':>6}")
    for row in payload["summary"]:
        print(f"{row['density']:>7} {row['controller']:>10} {row['runs']:>4} "
              f"{row['success_rate']:>7.1f}% {str(row['mean_time_s']):>7} "
              f"{str(row['mean_clearance_m']):>7} {str(row['mean_acc_rms']):>7} "
              f"{str(row['mean_jerk_rms']):>7} {str(row['edge_ratio']):>6}")
    print("* 成功回合均值；! 全部回合均值（失败按 60 s 超时计入）；"
          f"edge% 为成功回合中最小间隙 ≤{args.edge_threshold:.2f} m 的比例")
    print("\nresults:", os.path.abspath(args.out))
    print("figures:", os.path.abspath(figdir) if figdir else "skipped")


if __name__ == "__main__":
    main()
