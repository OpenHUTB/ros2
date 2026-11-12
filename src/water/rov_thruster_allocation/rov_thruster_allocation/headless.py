"""Offline experiment: pseudo-inverse versus saturation-aware allocation."""

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
from .core import (
    allocate_pseudo_inverse,
    allocate_qp,
    effort,
    error_norm,
    at_limit_ratio,
    overshoot_ratio,
)
from .scene import DemandConfig, make_demands


def _evaluate(wrenches, allocator, matrix, limit) -> dict:
    errors, efforts, at_limit, overshoot, peak = [], [], [], [], []
    for demand in wrenches:
        forces = allocator(matrix, demand, limit)
        errors.append(error_norm(matrix, forces, demand))
        efforts.append(effort(forces))
        at_limit.append(at_limit_ratio(forces, limit))
        overshoot.append(overshoot_ratio(forces, limit))
        peak.append(float(np.max(np.abs(forces))))
    errors = np.asarray(errors)
    return {
        "wrench_error_rms": float(np.sqrt(np.mean(errors**2))),
        "wrench_error_p95": float(np.percentile(errors, 95)),
        "wrench_error_max": float(np.max(errors)),
        "effort_mean": float(np.mean(efforts)),
        "at_limit_pct": float(100.0 * np.mean(at_limit)),
        "overshoot_pct": float(100.0 * np.mean(overshoot)),
        "max_abs_force_n": float(np.max(peak)),
        "samples": int(errors.size),
    }


def run_experiment(
    out_dir: Path | None = None,
    figure_dir: Path | None = None,
    make_plots: bool = False,
) -> dict:
    config = DemandConfig()
    data = make_demands(config)
    matrix = data["matrix"]
    limit = data["max_thrust"]

    allocators = {
        "pseudo_inverse": lambda m, d, l: allocate_pseudo_inverse(m, d),
        "clipped_pseudo_inverse": lambda m, d, l: allocate_pseudo_inverse(m, d, l),
        "qp_allocation": lambda m, d, l: allocate_qp(m, d, l),
    }
    summary = {
        "module": "rov_thruster_allocation",
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
        "configuration_matrix_condition": float(np.linalg.cond(matrix)),
        "inside": {},
        "outside": {},
    }
    for name, allocator in allocators.items():
        summary["inside"][name] = _evaluate(
            data["inside_wrenches"], allocator, matrix, limit
        )
        summary["outside"][name] = _evaluate(
            data["outside_wrenches"], allocator, matrix, limit
        )

    if out_dir is not None:
        out_dir.mkdir(parents=True, exist_ok=True)
        with (out_dir / "summary.json").open("w", encoding="utf-8", newline="\n") as stream:
            stream.write(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    if make_plots:
        if figure_dir is None:
            raise ValueError("figure_dir is required when make_plots is True")
        from . import plotting

        figure_dir.mkdir(parents=True, exist_ok=True)
        plotting.plot_errors(data, allocators, figure_dir / "wrench_error.png")
        plotting.plot_effort(summary, figure_dir / "effort.png")
    return summary


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("results"))
    parser.add_argument("--figure-dir", type=Path, default=None)
    parser.add_argument("--no-plots", action="store_true")
    options = parser.parse_args(args)
    summary = run_experiment(
        out_dir=options.out,
        figure_dir=options.figure_dir,
        make_plots=not options.no_plots and options.figure_dir is not None,
    )
    print(f"configuration matrix condition: {summary['configuration_matrix_condition']:.1f}")
    for scope in ("inside", "outside"):
        print(f"[{scope} envelope]")
        for name, block in summary[scope].items():
            print(
                f"  {name:<24} error RMS {block['wrench_error_rms']:8.2f} | "
                f"p95 {block['wrench_error_p95']:8.2f} | "
                f"effort {block['effort_mean']:8.1f} | at-limit {block['at_limit_pct']:5.1f} % | "
                f"over {block['overshoot_pct']:4.1f} % | max|f| {block['max_abs_force_n']:5.1f} N"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
