"""Offline experiment: identify friction, then compensate it in tracking."""

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
from .core import FrictionModel, identify_friction, torque_error
from .scene import JointConfig, simulate_sweeps, simulate_tracking

TRUTH = FrictionModel(coulomb=0.45, viscous=0.08, static=0.65, stribeck_velocity=0.20)


def run_experiment(
    out_dir: Path | None = None,
    figure_dir: Path | None = None,
    make_plots: bool = True,
) -> dict:
    config = JointConfig()
    sweeps = simulate_sweeps(config, TRUTH)
    model, info = identify_friction(sweeps["speeds"], sweeps["torques"])

    baseline = simulate_tracking(config, TRUTH)
    compensated = simulate_tracking(config, TRUTH, model=model)

    summary = {
        "module": "joint_friction_identification",
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
        "truth": TRUTH.to_dict(),
        "identified": model.to_dict(),
        "identification": {
            "residual_rms": info["residual_rms"],
            "grid_points": info["grid_points"],
            "error_pct": torque_error(model, TRUTH),
            "sweeps": {
                "speeds": sweeps["speeds"].tolist(),
                "torques": sweeps["torques"].tolist(),
            },
        },
        "tracking": {
            "no_compensation": {
                "rms_error_rad": baseline["rms_error_rad"],
                "max_error_rad": baseline["max_error_rad"],
            },
            "compensated": {
                "rms_error_rad": compensated["rms_error_rad"],
                "max_error_rad": compensated["max_error_rad"],
            },
            "improvement_pct": float(
                100.0 * (1.0 - compensated["rms_error_rad"] / baseline["rms_error_rad"])
            ),
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
        plotting.plot_friction(sweeps, model, figure_dir / "friction_curve.png")
        plotting.plot_tracking(baseline, compensated, figure_dir / "tracking.png")
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
    errors = summary["identification"]["error_pct"]
    print(
        f"identified coulomb {summary['identified']['coulomb']:.4f} "
        f"({errors['coulomb_pct']:+.1f} %), viscous {summary['identified']['viscous']:.4f} "
        f"({errors['viscous_pct']:+.1f} %), static {summary['identified']['static']:.4f} "
        f"({errors['static_pct']:+.1f} %), v_s {summary['identified']['stribeck_velocity']:.4f} "
        f"({errors['stribeck_pct']:+.1f} %)"
    )
    tracking = summary["tracking"]
    print(
        f"tracking RMS: {tracking['no_compensation']['rms_error_rad']:.5f} -> "
        f"{tracking['compensated']['rms_error_rad']:.5f} rad "
        f"({tracking['improvement_pct']:.1f} % better)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
