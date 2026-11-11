"""Offline experiment: velocity estimates from quantised encoder positions."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import datetime
import json
from pathlib import Path
import platform
import sys

import numpy as np

from . import __version__
from .core import alpha_beta, kalman_velocity, moving_average, raw_difference
from .scene import ProfileConfig, measure, trapezoid

RESOLUTIONS = (1024, 4096, 16384)


def _metrics(estimate, truth) -> dict:
    error = np.asarray(estimate) - np.asarray(truth)
    return {
        "rmse_rad_s": float(np.sqrt(np.mean(error**2))),
        "max_abs_rad_s": float(np.max(np.abs(error))),
        "bias_rad_s": float(np.mean(error)),
    }


def _lag_ms(estimate, truth, dt: float) -> float:
    """Cross-correlation lag of the estimate against the truth, in ms."""
    estimate = np.asarray(estimate, dtype=float)
    truth = np.asarray(truth, dtype=float)
    best_lag, best_correlation = 0, -np.inf
    for lag in range(-30, 31):
        if lag >= 0:
            left, right = truth[: truth.size - lag], estimate[lag:]
        else:
            left, right = truth[-lag:], estimate[: estimate.size + lag]
        if left.size < 20:
            continue
        correlation = float(np.corrcoef(left, right)[0, 1])
        if correlation > best_correlation:
            best_correlation, best_lag = correlation, lag
    return float(best_lag * dt * 1000.0)


def run_experiment(
    out_dir: Path | None = None,
    figure_dir: Path | None = None,
    make_plots: bool = True,
) -> dict:
    config = ProfileConfig()
    profile = trapezoid(config)
    truth = profile["velocity_rad_s"]
    summary = {
        "module": "encoder_quantization_velocity",
        "version": __version__,
        "generated_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(
            timespec="seconds"
        ),
        "environment": {
            "python": sys.version.split()[0],
            "numpy": np.__version__,
            "platform": platform.platform(),
        },
        "config": asdict(config),
        "resolutions": {},
    }
    for resolution in RESOLUTIONS:
        data = measure(profile, resolution)
        dt = data["dt"]
        raw = raw_difference(data["position_rad"], dt)
        smooth = moving_average(raw, 5)
        ab = alpha_beta(data["position_rad"], dt, alpha=0.35, beta=0.08)
        kf = kalman_velocity(data["position_rad"], dt,
                             process_sigma=25.0, measurement_sigma=data["step_rad"] / 3.0)
        summary["resolutions"][str(resolution)] = {
            "step_rad": data["step_rad"],
            "raw": {**_metrics(raw, truth), "lag_ms": _lag_ms(raw, truth, dt)},
            "moving_average": {**_metrics(smooth, truth), "lag_ms": _lag_ms(smooth, truth, dt)},
            "alpha_beta": {**_metrics(ab, truth), "lag_ms": _lag_ms(ab, truth, dt)},
            "kalman": {**_metrics(kf, truth), "lag_ms": _lag_ms(kf, truth, dt)},
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
        plotting.plot_velocity(profile, summary, figure_dir / "velocity_estimate.png")
        plotting.plot_resolution(summary, figure_dir / "resolution_sweep.png")
    return summary


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("results"))
    parser.add_argument("--figure-dir", type=Path, default=None)
    parser.add_argument("--no-plots", action="store_true")
    options = parser.parse_args(args)
    summary = run_experiment(
        out_dir=options.out, figure_dir=options.figure_dir, make_plots=not options.no_plots
    )
    for resolution, block in summary["resolutions"].items():
        print(f"[{resolution} CPR]")
        for name in ("raw", "moving_average", "alpha_beta", "kalman"):
            entry = block[name]
            print(
                f"  {name:<15} RMSE {entry['rmse_rad_s']:6.3f} rad/s | "
                f"max {entry['max_abs_rad_s']:6.2f} | lag {entry['lag_ms']:+6.1f} ms"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
