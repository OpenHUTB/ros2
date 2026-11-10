"""Offline experiment driver: metrics, CSV/JSON evidence and figures.

Run without ROS:

    python main.py --out results --figure-dir ../../docs/img/ground/uwb_range_odom_fusion

The command is deterministic: the same seeds and configuration always produce
the same tables, so a reviewer can diff ``results/summary.json`` against the
numbers quoted in the documentation.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
import datetime
import json
from pathlib import Path
import platform
import sys

import numpy as np

from . import __version__
from .runner import default_estimators, run_ekf, run_trilateration
from .scenario import ScenarioConfig, simulate

COLLINEAR_LAYOUT = ((-6.0, 0.2), (-2.0, 0.2), (2.0, 0.2), (6.0, 0.2))
GDOP_PROBES = {
    "arena centre (0, 0)": (0.0, 0.0),
    "straight section (4.5, 2.8)": (4.5, 2.8),
    "near anchor line (0, 0.6)": (0.0, 0.6),
    "inside pillar shadow (-1.2, 1.3)": (-1.2, 1.3),
}


def _metrics_by_name(simulation, runs) -> dict:
    return {run.name: run.metrics(simulation) for run in runs}


def _aggregate(per_seed_metrics: list[dict], names) -> dict:
    aggregate = {}
    for name in names:
        entries = [metrics[name] for metrics in per_seed_metrics if name in metrics]
        if not entries:
            continue
        rmse = np.array([entry["position_m"]["rmse"] for entry in entries])
        p95 = np.array([entry["position_m"]["p95"] for entry in entries])
        maximum = np.array([entry["position_m"]["max"] for entry in entries])
        final = np.array([entry["position_m"]["final"] for entry in entries])
        runtime = np.array([entry["runtime_per_epoch_us"] for entry in entries])
        block = {
            "rmse_mean_m": float(np.mean(rmse)),
            "rmse_std_m": float(np.std(rmse)),
            "p95_mean_m": float(np.mean(p95)),
            "max_mean_m": float(np.mean(maximum)),
            "final_mean_m": float(np.mean(final)),
            "runtime_us_mean": float(np.mean(runtime)),
            "seeds": len(entries),
        }
        if "heading_deg" in entries[0]:
            heading = np.array([entry["heading_deg"]["rmse"] for entry in entries])
            block["heading_rmse_mean_deg"] = float(np.mean(heading))
        if "nees" in entries[0]:
            nees = np.array([entry["nees"]["mean"] for entry in entries])
            block["nees_mean"] = float(np.mean(nees))
            block["nees_consistent_seeds"] = int(
                sum(entry["nees"]["consistent"] for entry in entries)
            )
        if "gating" in entries[0]:
            false_reject = np.array(
                [
                    100.0 * entry["gating"]["rejected_los"] / max(1, entry["gating"]["los_total"])
                    for entry in entries
                ]
            )
            detected = np.array(
                [
                    100.0
                    * entry["gating"]["rejected_nlos"]
                    / max(1, entry["gating"]["nlos_total"])
                    for entry in entries
                ]
            )
            accepted_nlos = np.array(
                [
                    100.0
                    * entry["gating"]["accepted_nlos"]
                    / max(1, entry["gating"]["nlos_total"])
                    for entry in entries
                ]
            )
            block["false_reject_pct"] = float(np.mean(false_reject))
            block["nlos_detected_pct"] = float(np.mean(detected))
            block["nlos_accepted_pct"] = float(np.mean(accepted_nlos))
        aggregate[name] = block
    return aggregate


def _per_seed_rows(per_seed_metrics: list[dict]) -> list[dict]:
    """One compact row per seed so every aggregate number can be traced back."""
    rows = []
    for seed, metrics in enumerate(per_seed_metrics):
        row = {"seed": seed}
        for name, entry in metrics.items():
            row[f"{name}_rmse_m"] = round(entry["position_m"]["rmse"], 4)
            if "nees" in entry:
                row[f"{name}_nees_mean"] = round(entry["nees"]["mean"], 3)
        rows.append(row)
    return rows


def _write_timeseries(simulation, runs, csv_path: Path) -> None:
    by_name = {run.name: run for run in runs}
    columns = ["time_s", "truth_x", "truth_y", "truth_theta", "odometry_x", "odometry_y",
               "trilateration_x", "trilateration_y", "ekf_ungated_x", "ekf_ungated_y",
               "ekf_gated_x", "ekf_gated_y", "nlos_any", "accepted", "rejected"]
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(columns)
        for index, time_s in enumerate(simulation.times):
            gated = by_name["ekf_gated"]
            writer.writerow(
                [
                    f"{time_s:.3f}",
                    f"{simulation.truth[index, 0]:.4f}",
                    f"{simulation.truth[index, 1]:.4f}",
                    f"{simulation.truth[index, 2]:.5f}",
                    f"{by_name['odometry_only'].positions[index, 0]:.4f}",
                    f"{by_name['odometry_only'].positions[index, 1]:.4f}",
                    f"{by_name['trilateration'].positions[index, 0]:.4f}",
                    f"{by_name['trilateration'].positions[index, 1]:.4f}",
                    f"{by_name['ekf_ungated'].positions[index, 0]:.4f}",
                    f"{by_name['ekf_ungated'].positions[index, 1]:.4f}",
                    f"{gated.positions[index, 0]:.4f}",
                    f"{gated.positions[index, 1]:.4f}",
                    int(np.any(simulation.nlos[index])),
                    int(np.sum(gated.accepted[index])),
                    int(np.sum(gated.rejected[index])),
                ]
            )


def run_experiment(
    seeds: int = 10,
    duration_s: float = 70.0,
    out_dir: Path | None = None,
    figure_dir: Path | None = None,
    make_plots: bool = True,
    make_gif: bool = True,
) -> dict:
    """Run all seeds, write evidence files and return the summary dictionary."""
    config = ScenarioConfig(duration_s=duration_s)
    per_seed = []
    first_simulation, first_runs = None, None
    for seed in range(seeds):
        simulation = simulate(config, seed=seed)
        runs = default_estimators(simulation)
        per_seed.append(_metrics_by_name(simulation, runs))
        if seed == 0:
            first_simulation, first_runs = simulation, runs

    names = [run.name for run in first_runs]
    summary = {
        "module": "uwb_range_odom_fusion",
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
            **{
                key: value
                for key, value in asdict(config).items()
                if key not in {"wheel"}
            },
            "wheel": asdict(config.wheel),
        },
        "seeds": list(range(seeds)),
        "main": _aggregate(per_seed, names),
        "gating_thresholds": {
            "gate_chi2_1dof": config.gate_chi2,
            "gate_probability": 0.99,
            "nominal_false_reject_pct": 1.0,
        },
        "geometry": _geometry_experiment(config, seeds),
        "per_seed": _per_seed_rows(per_seed),
        "nees": {
            "dof": 3,
            "interval_95_low": float(
                first_runs[-1].metrics(first_simulation)["nees"]["interval_low"]
            ),
            "interval_95_high": float(
                first_runs[-1].metrics(first_simulation)["nees"]["interval_high"]
            ),
        },
    }

    if out_dir is not None:
        out_dir.mkdir(parents=True, exist_ok=True)
        _write_timeseries(first_simulation, first_runs, out_dir / "timeseries.csv")
        (out_dir / "summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
            newline="\n",
        )

    if make_plots:
        if figure_dir is None:
            raise ValueError("figure_dir is required when make_plots is True")
        try:
            from . import plotting
        except ImportError as error:  # pragma: no cover
            raise RuntimeError("matplotlib is not available") from error
        figure_dir.mkdir(parents=True, exist_ok=True)
        plotting.plot_trajectories(first_simulation, first_runs, figure_dir / "trajectory.png")
        plotting.plot_errors(first_simulation, first_runs, figure_dir / "error_gating.png")
        plotting.plot_gdop(
            config, config.with_anchor_layout(COLLINEAR_LAYOUT), figure_dir / "gdop_map.png"
        )
        plotting.plot_nees(
            first_simulation,
            next(run for run in first_runs if run.name == "ekf_gated"),
            figure_dir / "nees_consistency.png",
        )
        if make_gif:
            plotting.animate(
                first_simulation,
                next(run for run in first_runs if run.name == "ekf_gated"),
                figure_dir / "demo.gif",
                fps=5.0,
                seconds=45.0,
            )
    return summary


def _geometry_experiment(config: ScenarioConfig, seeds: int) -> dict:
    from .geometry import gdop

    layouts = {
        "square": config.anchor_array,
        "collinear": np.asarray(COLLINEAR_LAYOUT, dtype=float),
    }
    result = {}
    for name, anchors in layouts.items():
        layout_config = config.with_anchor_layout(anchors)
        probes = {
            label: float(gdop(point, anchors)) for label, point in GDOP_PROBES.items()
        }
        ekf_rmse, trilat_rmse = [], []
        for seed in range(min(seeds, 5)):
            simulation = simulate(layout_config, seed=seed)
            ekf_rmse.append(run_ekf(simulation).metrics(simulation)["position_m"]["rmse"])
            trilat_rmse.append(
                run_trilateration(simulation).metrics(simulation)["position_m"]["rmse"]
            )
        result[name] = {
            "anchors": [list(map(float, anchor)) for anchor in anchors],
            "gdop": probes,
            "ekf_rmse_mean_m": float(np.mean(ekf_rmse)),
            "trilateration_rmse_mean_m": float(np.mean(trilat_rmse)),
            "seeds": min(seeds, 5),
        }
    return result


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, default=10, help="number of random seeds")
    parser.add_argument("--duration", type=float, default=70.0, help="run length in seconds")
    parser.add_argument("--out", type=Path, default=Path("results"), help="evidence directory")
    parser.add_argument("--figure-dir", type=Path, default=None, help="figure output directory")
    parser.add_argument("--no-plots", action="store_true", help="skip every matplotlib figure")
    parser.add_argument("--no-gif", action="store_true", help="skip the animated GIF")
    options = parser.parse_args(args)
    summary = run_experiment(
        seeds=options.seeds,
        duration_s=options.duration,
        out_dir=options.out,
        figure_dir=options.figure_dir,
        make_plots=not options.no_plots,
        make_gif=not options.no_gif,
    )
    print(f"{'estimator':<18}{'RMSE [m]':>12}{'p95 [m]':>10}{'max [m]':>10}{'final [m]':>11}")
    for name, block in summary["main"].items():
        print(
            f"{name:<18}{block['rmse_mean_m']:>12.4f}{block['p95_mean_m']:>10.3f}"
            f"{block['max_mean_m']:>10.3f}{block['final_mean_m']:>11.3f}"
        )
    print(json.dumps(summary["geometry"], ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
