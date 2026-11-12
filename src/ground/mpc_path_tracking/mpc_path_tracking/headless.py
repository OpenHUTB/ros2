"""Offline experiment: MPC tracking versus pure pursuit."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import datetime
import json
from pathlib import Path
import platform
import sys
import time

import numpy as np

from . import __version__
from .core import MpcConfig
from .scene import ReferencePath, TrackConfig, run_mpc, run_pure_pursuit, tracking_metrics


def run_experiment(
    out_dir: Path | None = None,
    figure_dir: Path | None = None,
    make_plots: bool = True,
    duration_s: float = 40.0,
) -> dict:
    path = ReferencePath()
    track = TrackConfig(duration_s=duration_s)
    mpc_config = MpcConfig()

    started = time.perf_counter()
    mpc_run = run_mpc(path, track, mpc_config)
    mpc_elapsed = time.perf_counter() - started
    started = time.perf_counter()
    pursuit_run = run_pure_pursuit(path, track)
    pursuit_elapsed = time.perf_counter() - started

    mpc_metrics = tracking_metrics(path, mpc_run)
    mpc_metrics["solve_ms_per_step"] = 1000.0 * mpc_elapsed / max(1, mpc_metrics["steps"])
    pursuit_metrics = tracking_metrics(path, pursuit_run)
    pursuit_metrics["solve_ms_per_step"] = 1000.0 * pursuit_elapsed / max(1, pursuit_metrics["steps"])

    summary = {
        "module": "mpc_path_tracking",
        "version": __version__,
        "generated_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(
            timespec="seconds"
        ),
        "environment": {
            "python": sys.version.split()[0],
            "numpy": np.__version__,
            "platform": platform.platform(),
        },
        "config": {"track": asdict(track), "mpc": asdict(mpc_config)},
        "mpc": mpc_metrics,
        "pure_pursuit": pursuit_metrics,
    }
    if out_dir is not None:
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    if make_plots:
        if figure_dir is None:
            raise ValueError("figure_dir is required when make_plots is True")
        from . import plotting

        figure_dir.mkdir(parents=True, exist_ok=True)
        plotting.plot_tracking(path, mpc_run, pursuit_run, figure_dir / "tracking.png")
        plotting.plot_controls(mpc_run, pursuit_run, figure_dir / "controls.png")
    return summary


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("results"))
    parser.add_argument("--figure-dir", type=Path, default=None)
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument("--duration", type=float, default=40.0)
    options = parser.parse_args(args)
    summary = run_experiment(
        out_dir=options.out,
        figure_dir=options.figure_dir,
        make_plots=not options.no_plots,
        duration_s=options.duration,
    )
    for name in ("pure_pursuit", "mpc"):
        block = summary[name]
        print(
            f"{name:<13} RMSE {block['lateral_rmse_m']:.3f} m | max {block['lateral_max_m']:.3f} m | "
            f"yaw-rate RMS {block['yaw_rate_rms_rad_s']:.3f} | "
            f"jerk {block['yaw_rate_jerk_rms']:.3f} | {block['solve_ms_per_step']:.2f} ms/step"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
