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

    # Ten independent realisations: the +16 % bias error of one seed is not a
    # method error, so report the spread instead of a single draw.
    arw_errors, bias_errors = [], []
    for repeat in range(10):
        sample = simulate_static(
            ImuConfig(
                static_duration_s=static_s,
                sweep_duration_s=sweep_s,
                noise=TRUTH,
                seed=repeat,
            )
        )
        repeat_taus, repeat_adev = allan_deviation(sample["rate_deg_s"], sample["dt"])
        repeat_model = fit_noise_model(repeat_taus, repeat_adev)
        arw_errors.append(100.0 * (repeat_model.arw - TRUTH.arw) / TRUTH.arw)
        bias_errors.append(
            100.0
            * (repeat_model.bias_instability - TRUTH.bias_instability)
            / TRUTH.bias_instability
        )
    seed_stats = {
        "arw_pct": {
            "median": float(np.median(arw_errors)),
            "low": float(np.percentile(arw_errors, 2.5)),
            "high": float(np.percentile(arw_errors, 97.5)),
        },
        "bias_pct": {
            "median": float(np.median(bias_errors)),
            "low": float(np.percentile(bias_errors, 2.5)),
            "high": float(np.percentile(bias_errors, 97.5)),
        },
        "seeds": len(arw_errors),
    }

    sweep = simulate_tilt_sweep(config)
    tuned = run_filter(sweep, identified)
    naive = run_filter(sweep, NAIVE)
    # Ablation: which identified parameter actually moves the filter.
    arw_only = run_filter(sweep, NoiseModel(arw=identified.arw * 0.25,
                                            bias_instability=identified.bias_instability))
    bias_only = run_filter(sweep, NoiseModel(arw=identified.arw,
                                             bias_instability=identified.bias_instability * 60.0))

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
            "seed_stats": seed_stats,
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
            "ablation_arw_only": {"rmse_deg": arw_only["rmse_deg"],
                                   "nees_mean": arw_only["nees_mean"]},
            "ablation_bias_x60": {"rmse_deg": bias_only["rmse_deg"],
                                    "nees_mean": bias_only["nees_mean"]},
        },
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
    for name in ("allan_tuned", "naive_tuned"):
        block = summary["filters"][name]
        print(
            f"{name:<14} RMSE {block['rmse_deg']:.4f} deg | "
            f"max {block['max_error_deg']:.4f} deg | NEES {block['nees_mean']:.2f}"
        )
    for name in ("ablation_arw_only", "ablation_bias_x60"):
        block = summary["filters"][name]
        print(f"{name:<14} RMSE {block['rmse_deg']:.4f} deg | NEES {block['nees_mean']:.2f}")
    stats = summary["identification"]["seed_stats"]
    print(
        f"10 seeds: ARW median {stats['arw_pct']['median']:+.2f} % "
        f"[{stats['arw_pct']['low']:+.2f}, {stats['arw_pct']['high']:+.2f}], "
        f"BI median {stats['bias_pct']['median']:+.2f} % "
        f"[{stats['bias_pct']['low']:+.2f}, {stats['bias_pct']['high']:+.2f}]"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
