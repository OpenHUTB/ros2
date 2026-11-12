"""Offline experiment: wind estimate versus manoeuvre and noise level."""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import datetime
import json
from pathlib import Path
import platform
import sys

import numpy as np

from . import __version__
from .core import estimate_wind, observability, reciprocal_estimate
from .scene import MANEUVERS, WindConfig, simulate_maneuver

NOISE_GRID = (0.05, 0.10, 0.20, 0.40)


def _speed_error(estimate, truth) -> float:
    return float(np.linalg.norm(np.asarray(estimate) - np.asarray(truth)))


def _direction_error(estimate, truth) -> float:
    estimate = np.asarray(estimate, dtype=float)
    truth = np.asarray(truth, dtype=float)
    difference = np.arctan2(
        np.sin(np.arctan2(estimate[1], estimate[0]) - np.arctan2(truth[1], truth[0])),
        np.cos(np.arctan2(estimate[1], estimate[0]) - np.arctan2(truth[1], truth[0])),
    )
    return float(abs(np.rad2deg(difference)))


def run_experiment(
    out_dir: Path | None = None,
    figure_dir: Path | None = None,
    make_plots: bool = True,
    trials: int = 40,
) -> dict:
    base = WindConfig()
    summary = {
        "module": "uav_wind_estimation",
        "version": __version__,
        "generated_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(
            timespec="seconds"
        ),
        "environment": {
            "python": sys.version.split()[0],
            "numpy": np.__version__,
            "platform": platform.platform(),
        },
        "config": asdict(base),
        "trials": trials,
        "maneuvers": {},
        "noise_sweep": {},
    }

    for maneuver in MANEUVERS:
        info = observability(simulate_maneuver(base, maneuver)["headings"])
        speed_errors, direction_errors, naive_speed_errors = [], [], []
        for seed in range(trials):
            data = simulate_maneuver(replace(base, seed=seed), maneuver)
            try:
                estimate = estimate_wind(
                    data["ground_velocity"], data["headings"], sigma_m_s=base.gps_noise_m_s
                )
                speeds = estimate.wind
            except ValueError:
                speeds = None
            naive = reciprocal_estimate(data["ground_velocity"], data["headings"])
            naive_speed_errors.append(_speed_error(naive, base.wind))
            if speeds is None:
                speed_errors.append(float("nan"))
                direction_errors.append(float("nan"))
            else:
                speed_errors.append(_speed_error(speeds, base.wind))
                direction_errors.append(_direction_error(speeds, base.wind))
        summary["maneuvers"][maneuver] = {
            "observability": info,
            "ls_speed_rmse_m_s": float(np.sqrt(np.nanmean(np.square(speed_errors)))) if not np.all(np.isnan(speed_errors)) else None,
            "ls_direction_rmse_deg": float(np.sqrt(np.nanmean(np.square(direction_errors)))) if not np.all(np.isnan(direction_errors)) else None,
            "reciprocal_speed_rmse_m_s": float(np.sqrt(np.mean(np.square(naive_speed_errors)))),
            "estimable": bool(not np.all(np.isnan(speed_errors))),
        }

    for sigma in NOISE_GRID:
        config = replace(base, gps_noise_m_s=sigma)
        errors = []
        for seed in range(trials):
            data = simulate_maneuver(replace(config, seed=seed), "circle")
            estimate = estimate_wind(data["ground_velocity"], data["headings"], sigma_m_s=sigma)
            errors.append(_speed_error(estimate.wind, base.wind))
        summary["noise_sweep"][f"{sigma:.2f}"] = {
            "gps_noise_m_s": sigma,
            "speed_rmse_m_s": float(np.sqrt(np.mean(np.square(errors)))),
        }

    if out_dir is not None:
        out_dir.mkdir(parents=True, exist_ok=True)
        with (out_dir / "summary.json").open("w", encoding="utf-8", newline="\n") as stream:
            stream.write(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    if make_plots:
        if figure_dir is None:
            raise ValueError("figure_dir is required when make_plots is True")
        from . import plotting

        figure_dir.mkdir(parents=True, exist_ok=True)
        plotting.plot_maneuvers(base, figure_dir / "maneuvers.png")
        plotting.plot_noise(summary["noise_sweep"], figure_dir / "noise_sweep.png")
    return summary


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("results"))
    parser.add_argument("--figure-dir", type=Path, default=None)
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument("--trials", type=int, default=40)
    options = parser.parse_args(args)
    summary = run_experiment(
        out_dir=options.out,
        figure_dir=options.figure_dir,
        make_plots=not options.no_plots,
        trials=options.trials,
    )
    for name, block in summary["maneuvers"].items():
        speed = block["ls_speed_rmse_m_s"]
        direction = block["ls_direction_rmse_deg"]
        print(
            f"{name:<11} LS speed '{(speed if speed is not None else float('nan')):.3f}' m/s | "
            f"direction {(direction if direction is not None else float('nan')):.2f} deg | "
            f"naive {(block['reciprocal_speed_rmse_m_s']):.3f} m/s | "
            f"observable {block['observability']['observable']}"
        )
    for key, block in summary["noise_sweep"].items():
        print(f"  gps sigma {key}: speed RMSE {block['speed_rmse_m_s']:.3f} m/s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
