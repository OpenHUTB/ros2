"""Offline experiment: estimate the visual delay, then measure what it buys."""

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
from .core import align, estimate_offset, residual_rms
from .scene import SignalConfig, simulate_pair


def _trial(config: SignalConfig) -> dict:
    pair = simulate_pair(config)
    estimate = estimate_offset(pair["gyro"], pair["visual"], pair["dt"])
    return {
        "true_offset_s": pair["true_offset_s"],
        "estimated_offset_s": estimate.offset_s,
        "error_ms": 1000.0 * (estimate.offset_s - pair["true_offset_s"]),
        "peak_correlation": estimate.peak_correlation,
        "peak_z": estimate.peak_z,
        "residual_before": residual_rms(
            pair["gyro"], pair["visual"], 0.0, pair["dt"], config.visual_gain
        ),
        "residual_after": residual_rms(
            pair["gyro"], pair["visual"], estimate.offset_s, pair["dt"], config.visual_gain
        ),
        "residual_oracle": residual_rms(
            pair["gyro"], pair["visual"], pair["true_offset_s"], pair["dt"], config.visual_gain
        ),
    }


def run_experiment(
    out_dir: Path | None = None,
    figure_dir: Path | None = None,
    make_plots: bool = True,
    trials: int = 200,
    truth_offset_s: float = 0.025,
) -> dict:
    base = SignalConfig(offset_s=truth_offset_s)
    rows = []
    for index in range(trials):
        config = SignalConfig(offset_s=truth_offset_s, seed=index)
        rows.append(_trial(config))

    errors = np.array([row["error_ms"] for row in rows])
    before = np.array([row["residual_before"] for row in rows])
    after = np.array([row["residual_after"] for row in rows])
    oracle = np.array([row["residual_oracle"] for row in rows])
    summary = {
        "module": "vio_time_offset_estimation",
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
        "offset_error_ms": {
            "mean": float(np.mean(errors)),
            "bias": float(np.mean(errors)),
            "rmse": float(np.sqrt(np.mean(errors**2))),
            "p95_abs": float(np.percentile(np.abs(errors), 95)),
            "max_abs": float(np.max(np.abs(errors))),
        },
        "residual": {
            "before_rms": float(np.mean(before)),
            "after_rms": float(np.mean(after)),
            "oracle_rms": float(np.mean(oracle)),
            "improvement_pct": float(100.0 * (1.0 - np.mean(after) / np.mean(before))),
        },
        "quality": {
            "peak_correlation_mean": float(np.mean([row["peak_correlation"] for row in rows])),
            "peak_z_mean": float(np.mean([row["peak_z"] for row in rows])),
        },
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
        pair = simulate_pair(base)
        plotting.plot_signals(pair, figure_dir / "signals.png")
        plotting.plot_errors(rows, figure_dir / "offset_error.png")
    return summary


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("results"))
    parser.add_argument("--figure-dir", type=Path, default=None)
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument("--trials", type=int, default=200)
    parser.add_argument("--offset", type=float, default=0.025)
    options = parser.parse_args(args)
    summary = run_experiment(
        out_dir=options.out,
        figure_dir=options.figure_dir,
        make_plots=not options.no_plots,
        trials=options.trials,
        truth_offset_s=options.offset,
    )
    block = summary["offset_error_ms"]
    residual = summary["residual"]
    print(
        f"offset error: mean {block['mean']:+.2f} ms | RMSE {block['rmse']:.2f} ms | "
        f"p95 |e| {block['p95_abs']:.2f} ms | max {block['max_abs']:.2f} ms"
    )
    print(
        f"residual: before {residual['before_rms']:.4f} -> after {residual['after_rms']:.4f} "
        f"(oracle {residual['oracle_rms']:.4f}), improvement {residual['improvement_pct']:.1f} %"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
