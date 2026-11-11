"""Offline experiment: calibrate the magnetometer, then evaluate headings."""

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
from .core import HeadingFilter, fit_ellipsoid, fit_hard_iron, magnetic_heading
from .scene import MagConfig, simulate_calibration, simulate_heading


def _heading_errors(estimate, truth) -> dict:
    error = np.arctan2(np.sin(estimate - truth), np.cos(estimate - truth))
    return {
        "rmse_deg": float(np.rad2deg(np.sqrt(np.mean(error**2)))),
        "max_deg": float(np.rad2deg(np.max(np.abs(error)))),
        "bias_deg": float(np.rad2deg(np.mean(error))),
    }


def run_experiment(
    out_dir: Path | None = None,
    figure_dir: Path | None = None,
    make_plots: bool = True,
    seed: int = 0,
) -> dict:
    config = MagConfig(seed=seed)
    calibration = simulate_calibration(config)
    fitted = fit_ellipsoid(calibration["measured_ut"], config.field_norm_ut)
    hard_only = fit_hard_iron(calibration["measured_ut"])

    truth_offset = calibration["hard_iron"]
    truth_matrix = np.linalg.inv(calibration["soft_matrix"])
    matrix_error = float(
        np.linalg.norm(fitted.matrix - truth_matrix) / np.linalg.norm(truth_matrix)
    )

    heading = simulate_heading(config)
    raw_heading = magnetic_heading(heading["measured_ut"], heading["acceleration"])
    hard_corrected = magnetic_heading(
        heading["measured_ut"] - hard_only, heading["acceleration"]
    )
    calibrated = fitted.apply(heading["measured_ut"])
    calibrated_heading = magnetic_heading(calibrated, heading["acceleration"])

    filter_ = HeadingFilter.create(
        initial_heading=float(calibrated_heading[0]),
        initial_sigma=np.deg2rad(10.0),
        gyro_sigma=np.deg2rad(0.5),
        mag_sigma=np.deg2rad(2.0),
    )
    fused = np.empty_like(calibrated_heading)
    for index in range(len(fused)):
        if index > 0:
            filter_.predict(float(heading["gyro_rad_s"][index - 1]), float(heading["dt"]))
        filter_.update(float(calibrated_heading[index]))
        fused[index] = filter_.state

    truth_yaw = heading["yaw_rad"]
    summary = {
        "module": "mag_hard_soft_iron_calibration",
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
            "field_norm_ut": config.field_norm_ut,
            "inclination_deg": config.inclination_deg,
            "hard_iron_ut": list(config.hard_iron_ut),
            "soft_diagonal": list(config.soft_diagonal),
            "soft_cross": config.soft_cross,
            "noise_ut": config.noise_ut,
            "calibration_samples": config.calibration_samples,
            "heading_rate_hz": config.heading_rate_hz,
            "heading_duration_s": config.heading_duration_s,
        },
        "calibration": {
            "offset_error_ut": float(np.linalg.norm(fitted.offset - truth_offset)),
            "matrix_relative_error": matrix_error,
            "field_norm_ut": float(fitted.field_norm),
            "residual_rms_ut": float(fitted.residual_rms),
            "hard_iron_only_offset": hard_only.tolist(),
            "fitted": fitted.to_dict(),
        },
        "heading": {
            "raw": _heading_errors(raw_heading, truth_yaw),
            "hard_iron_only": _heading_errors(hard_corrected, truth_yaw),
            "ellipsoid_calibrated": _heading_errors(calibrated_heading, truth_yaw),
            "fused": _heading_errors(fused, truth_yaw),
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
        plotting.plot_sphere(
            calibration["measured_ut"],
            fitted.apply(calibration["measured_ut"]),
            figure_dir / "calibration_sphere.png",
        )
        plotting.plot_heading(
            heading["time_s"],
            raw_heading,
            calibrated_heading,
            fused,
            truth_yaw,
            figure_dir / "heading_error.png",
        )
    return summary


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("results"))
    parser.add_argument("--figure-dir", type=Path, default=None)
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument("--seed", type=int, default=0)
    options = parser.parse_args(args)
    summary = run_experiment(
        out_dir=options.out,
        figure_dir=options.figure_dir,
        make_plots=not options.no_plots,
        seed=options.seed,
    )
    calibration = summary["calibration"]
    print(
        f"offset error {calibration['offset_error_ut']:.2f} uT | "
        f"matrix error {100.0 * calibration['matrix_relative_error']:.1f} % | "
        f"residual {calibration['residual_rms_ut']:.2f} uT"
    )
    for name, block in summary["heading"].items():
        print(
            f"{name:<20} RMSE {block['rmse_deg']:7.2f} deg | "
            f"max {block['max_deg']:7.2f} deg | bias {block['bias_deg']:+6.2f} deg"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
