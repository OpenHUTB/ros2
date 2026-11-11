"""Float vs fixed RTK baseline: success rate, wrong fixes and noise sensitivity."""

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
from .core import decorrelate, fix, float_solution
from .scene import RtkConfig, simulate, stack_epochs

WINDOW_GRID = (2, 5, 10, 20, 40)


def window_blocks(epochs: list[dict], size: int) -> list[dict]:
    """Re-block a run into fix windows of `size` epochs."""
    return [
        stack_epochs(epochs[start : start + size])
        for start in range(0, len(epochs) - size + 1, size)
    ]


def evaluate_block(block: dict, sigma_phase: float) -> dict:
    """Float and fixed baseline error for one fix window."""
    solution = float_solution(
        block["design"],
        block["phase_cycles"],
        block["code_m"],
        sigma_phase_cycles=sigma_phase,
        ambiguity_count=len(block["ambiguity"]),
    )
    transform = decorrelate(solution.cov_nn)
    result = fix(solution, transform)
    truth = block["baseline"]
    return {
        "float_error_m": float(np.linalg.norm(solution.baseline - truth)),
        "fixed_error_m": float(np.linalg.norm(result.baseline - truth)),
        "correct": bool(np.array_equal(result.integer_ambiguity, block["ambiguity"])),
        "fixed": bool(result.fixed),
        "ratio": float(result.ratio),
        "float_ambiguity_sigma": float(np.sqrt(np.mean(np.diag(solution.cov_nn)))),
        "search_candidates": int(result.candidates),
    }


def summarize(rows: list[dict]) -> dict:
    float_errors = np.array([row["float_error_m"] for row in rows])
    fixed_errors = np.array([row["fixed_error_m"] for row in rows])
    correct = np.array([row["correct"] for row in rows])
    fixed = np.array([row["fixed"] for row in rows])
    fixed_only = fixed_errors[fixed]
    fixed_rmse = (
        float(1000.0 * np.sqrt(np.mean(fixed_only**2))) if fixed_only.size else float("nan")
    )
    return {
        "attempts": len(rows),
        "float_rmse_mm": float(1000.0 * np.sqrt(np.mean(float_errors**2))),
        "fixed_rmse_mm": fixed_rmse,
        "fixed_epochs": int(fixed_only.size),
        "float_max_mm": float(1000.0 * np.max(float_errors)),
        "fixed_max_mm": float(1000.0 * np.max(fixed_errors)),
        "fixed_max_fixed_only_mm": float(1000.0 * np.max(fixed_only)) if fixed_only.size else float("nan"),
        "float_sigma_cycles_max": float(np.max([row["float_ambiguity_sigma"] for row in rows])),
        "success_rate_pct": float(100.0 * np.mean(correct)),
        "fix_rate_pct": float(100.0 * np.mean(fixed)),
        "wrong_fix_pct": float(100.0 * np.mean(fixed & ~correct)),
        "ratio_median": float(np.median([row["ratio"] for row in rows])),
        "float_sigma_cycles": float(np.mean([row["float_ambiguity_sigma"] for row in rows])),
        "candidates_median": float(np.median([row["search_candidates"] for row in rows])),
    }


def run_experiment(
    out_dir: Path | None = None,
    figure_dir: Path | None = None,
    make_plots: bool = True,
    seed: int = 0,
) -> dict:
    base_config = RtkConfig(seed=seed)
    epochs_data = simulate(base_config)
    rows = [
        evaluate_block(block, base_config.sigma_phase_cycles)
        for block in window_blocks(epochs_data, base_config.epochs_per_fix)
    ]
    main = summarize(rows)

    sweep = {}
    for size in WINDOW_GRID:
        sweep[str(size)] = summarize(
            [
                evaluate_block(block, base_config.sigma_phase_cycles)
                for block in window_blocks(epochs_data, size)
            ]
        )

    sample = window_blocks(epochs_data, base_config.epochs_per_fix)[0]
    sample_solution = float_solution(
        sample["design"],
        sample["phase_cycles"],
        sample["code_m"],
        sigma_phase_cycles=base_config.sigma_phase_cycles,
        ambiguity_count=len(sample["ambiguity"]),
    )
    transform = decorrelate(sample_solution.cov_nn)
    summary = {
        "module": "rtk_ambiguity_resolution",
        "version": __version__,
        "generated_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(
            timespec="seconds"
        ),
        "environment": {
            "python": sys.version.split()[0],
            "numpy": np.__version__,
            "platform": platform.platform(),
        },
        "config": asdict(base_config),
        "main": main,
        "window_sweep": sweep,
        "decorrelation": {
            "transform": transform.tolist(),
            "determinant": int(round(np.linalg.det(transform.astype(float)))),
            "condition_before": float(np.linalg.cond(sample_solution.cov_nn)),
            "condition_after": float(
                np.linalg.cond(transform @ sample_solution.cov_nn @ transform.T)
            ),
            "diag_before": [float(value) for value in np.diag(sample_solution.cov_nn)],
            "diag_after": [
                float(value)
                for value in np.diag(transform @ sample_solution.cov_nn @ transform.T)
            ],
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
        plotting.plot_error_cdf(rows, figure_dir / "baseline_error_cdf.png")
        plotting.plot_window_sweep(sweep, figure_dir / "success_vs_window.png")
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
    main_block = summary["main"]
    print(
        f"attempts {main_block['attempts']} | "
        f"float RMSE {main_block['float_rmse_mm']:.2f} mm | "
        f"fixed RMSE {main_block['fixed_rmse_mm']:.2f} mm | "
        f"success {main_block['success_rate_pct']:.1f} % | "
        f"wrong fix {main_block['wrong_fix_pct']:.1f} % | "
        f"float sigma {main_block['float_sigma_cycles']:.2f} cycles"
    )
    for window, block in summary["window_sweep"].items():
        print(
            f"  {window:>2} epochs/fix: success {block['success_rate_pct']:5.1f} % | "
            f"wrong {block['wrong_fix_pct']:5.1f} % | fixed RMSE {block['fixed_rmse_mm']:7.2f} mm"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
