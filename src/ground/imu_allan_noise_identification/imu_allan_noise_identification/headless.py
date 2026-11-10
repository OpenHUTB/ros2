"""Offline experiment: identify the noise model, then compare filter tuning."""

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
from .core import NoiseModel, allan_deviation, fit_noise_model
from .filters import run_filter
from .scene import ImuConfig, simulate_static, simulate_tilt_sweep

TRUTH = NoiseModel(arw=0.30, bias_instability=3.00)
NAIVE = NoiseModel(arw=0.075, bias_instability=0.75)  # 4x optimistic on purpose


def _curve(taus, adev, points: int = 24) -> list[dict]:
    if len(taus) <= points:
        selection = np.arange(len(taus))
    else:
        selection = np.unique(np.linspace(0, len(taus) - 1, points).astype(int))
    return [
        {"tau_s": float(taus[index]), "adev_deg_s": float(adev[index])}
        for index in selection
    ]


def run_experiment(
    out_dir: Path | None = None,
    figure_dir: Path | None = None,
    make_plots: bool = True,
    static_s: float = 3600.0,
    sweep_s: float = 300.0,
    seed: int = 0,
) -> dict:
    config = ImuConfig(
        static_duration_s=static_s, sweep_duration_s=sweep_s, noise=TRUTH, seed=seed
    )
    static = simulate_static(config)
    taus, adev = allan_deviation(static["rate_deg_s"], static["dt"])
    identified = fit_noise_model(taus, adev)
    errors = {
        "arw_pct": 100.0 * (identified.arw - TRUTH.arw) / TRUTH.arw,
        "bias_pct": 100.0
        * (identified.bias_instability - TRUTH.bias_instability)
        / TRUTH.bias_instability,
    }

    sweep = simulate_tilt_sweep(config)
    tuned = run_filter(sweep, identified)
    naive = run_filter(sweep, NAIVE)

    summary = {
        "module": "imu_allan_noise_identification",
        "version": __version__,
        "generated_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(
            timespec="seconds"
        ),
        "environment": {
            "python": sys.version.split()[0],
            "numpy": np.__version__,
            "platform": platform.platform(),
        },
        "config": {
            "rate_hz": config.rate_hz,
            "static_duration_s": static_s,
            "sweep_duration_s": sweep_s,
            "sweep_amplitude_deg": config.sweep_amplitude_deg,
            "sweep_frequency_hz": config.sweep_frequency_hz,
            "accel_noise_deg": config.accel_noise_deg,
            "constant_bias_deg_s": config.constant_bias_deg_s,
        },
        "identification": {
            "truth": asdict(TRUTH),
            "estimated": asdict(identified),
            "error_pct": errors,
            "samples": int(len(static["rate_deg_s"])),
            "allan_curve": _curve(taus, adev),
        },
        "filters": {
            "allan_tuned": {
                "rmse_deg": tuned["rmse_deg"],
                "max_error_deg": tuned["max_error_deg"],
                "nees_mean": tuned["nees_mean"],
            },
            "naive_tuned": {
                "model": asdict(NAIVE),
                "rmse_deg": naive["rmse_deg"],
                "max_error_deg": naive["max_error_deg"],
                "nees_mean": naive["nees_mean"],
            },
        },
    }
    if out_dir is not None:
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
            newline="\n",
        )
    if make_plots:
        if figure_dir is None:
            raise ValueError("figure_dir is required when make_plots is True")
        from . import plotting

        figure_dir.mkdir(parents=True, exist_ok=True)
        plotting.plot_allan(taus, adev, identified, TRUTH, figure_dir / "allan_deviation.png")
        plotting.plot_filters(sweep, tuned, naive, figure_dir / "filter_consistency.png")
    return summary


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("results"))
    parser.add_argument("--figure-dir", type=Path, default=None)
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument("--static-s", type=float, default=3600.0)
    parser.add_argument("--sweep-s", type=float, default=300.0)
    parser.add_argument("--seed", type=int, default=0)
    options = parser.parse_args(args)
    summary = run_experiment(
        out_dir=options.out,
        figure_dir=options.figure_dir,
        make_plots=not options.no_plots,
        static_s=options.static_s,
        sweep_s=options.sweep_s,
        seed=options.seed,
    )
    identified = summary["identification"]["estimated"]
    errors = summary["identification"]["error_pct"]
    print(
        f"identified ARW {identified['arw']:.4f} deg/sqrt(h) ({errors['arw_pct']:+.1f} %), "
        f"B {identified['bias_instability']:.4f} deg/h ({errors['bias_pct']:+.1f} %)"
    )
    for name, block in summary["filters"].items():
        print(
            f"{name:<14} RMSE {block['rmse_deg']:.4f} deg | "
            f"max {block['max_error_deg']:.4f} deg | NEES {block['nees_mean']:.2f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
