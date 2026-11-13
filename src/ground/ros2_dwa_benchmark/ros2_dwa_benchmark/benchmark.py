"""Random-scene stress benchmark for the A*+DWA and pure-DWA planners."""

from __future__ import annotations

import json
import os
from collections import defaultdict
from typing import Dict, List, Sequence

import numpy as np

try:
    from ros2_dwa_avoidance.hybrid_planner import AStarDWAController
    from ros2_dwa_avoidance.sim import DWAController, build_params, simulate
except ModuleNotFoundError as exc:
    if not (exc.name or "").startswith("ros2_dwa_avoidance"):
        raise
    import sys
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..",
                                    "ros2_dwa_avoidance"))
    from ros2_dwa_avoidance.hybrid_planner import AStarDWAController
    from ros2_dwa_avoidance.sim import DWAController, build_params, simulate

if __package__:
    from .metrics import summarize
    from .worlds import WALL_Y, generate_world
else:  # direct execution: python main_benchmark.py
    from metrics import summarize
    from worlds import WALL_Y, generate_world

DENSITIES = (4, 7, 10, 13)
WORLDS_PER_DENSITY = 10
CONTROLLERS = ("astar_dwa", "dwa")


def _make_controller(name: str, params):
    if name == "astar_dwa":
        controller = AStarDWAController(
            params, (-2.0, -WALL_Y - 1.0, 14.0, WALL_Y + 1.0))
        controller.name = "astar_dwa"
        return controller
    controller = DWAController(params)
    controller.name = "dwa"
    return controller


def run_benchmark(out_path: str, figure_dir: str,
                  densities: Sequence[int] = DENSITIES,
                  worlds_per_density: int = WORLDS_PER_DENSITY,
                  edge_threshold: float = 0.31) -> Dict[str, object]:
    params = build_params()
    rows: List[dict] = []
    skipped = 0

    for density in densities:
        for index in range(worlds_per_density):
            seed = 1000 + density * 100 + index
            scenario = generate_world(seed, density, params)
            if scenario is None:
                skipped += 1
                continue
            for name in CONTROLLERS:
                controller = _make_controller(name, params)
                result = simulate(scenario, controller, params)
                row = {
                    "density": density,
                    "world": scenario.name,
                    "seed": seed,
                    "controller": name,
                    **summarize(result, params.dt),
                }
                rows.append(row)
                print(f"{scenario.name:20s} {name:10s} "
                      f"success={row['success']} time={row['time_s']:5.1f} "
                      f"clear={row['min_clearance_m']:5.2f} "
                      f"acc={row['linear_acc_rms']:.3f}", flush=True)

    summaries = _summarize(rows, edge_threshold)
    if figure_dir:
        os.makedirs(figure_dir, exist_ok=True)
        _plot_success(rows, summaries, figure_dir)
        _plot_clearance(rows, figure_dir)
        _plot_comfort(rows, summaries, figure_dir)

    payload = {
        "config": {
            "densities": list(densities),
            "worlds_per_density": worlds_per_density,
            "controllers": list(CONTROLLERS),
            "params": params.__dict__,
            "skipped_infeasible": skipped,
        },
        "rows": rows,
        "summary": summaries,
    }
    with open(out_path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
    return payload


def _summarize(rows: Sequence[dict], edge_threshold: float = 0.31) -> List[dict]:
    """Aggregate per density/controller.

    Time and path are only defined for successful runs; acceleration, jerk and
    clearance are averaged over *all* runs (failed runs included, with their
    60 s timeout), so the table cannot cherry-pick the easy episodes.
    """
    grouped: Dict[tuple, List[dict]] = defaultdict(list)
    for row in rows:
        grouped[(row["density"], row["controller"])].append(row)

    out = []
    for (density, controller), group in sorted(grouped.items()):
        successes = [r for r in group if r["success"]]
        edge = [r for r in successes if r["min_clearance_m"] <= edge_threshold]
        out.append({
            "density": density,
            "controller": controller,
            "runs": len(group),
            "success_rate": round(100.0 * len(successes) / len(group), 1),
            "mean_time_s": round(float(np.mean([r["time_s"] for r in successes])), 2)
            if successes else None,
            "mean_time_all_s": round(float(np.mean([r["time_s"] for r in group])), 2),
            "mean_path_m": round(float(np.mean([r["path_length_m"] for r in successes])), 2)
            if successes else None,
            "mean_clearance_m": round(float(np.mean([r["min_clearance_m"] for r in group])), 3),
            "mean_acc_rms": round(float(np.mean([r["linear_acc_rms"] for r in group])), 4),
            "mean_jerk_rms": round(float(np.mean([r["jerk_rms"] for r in group])), 4),
            "edge_ratio": round(100.0 * len(edge) / len(successes), 1) if successes else None,
        })
    return out


def _plot_success(rows, summaries, figure_dir: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    densities = sorted({r["density"] for r in rows})
    width = 0.36
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    for offset, controller in ((-width / 2, "astar_dwa"), (width / 2, "dwa")):
        rates = []
        for density in densities:
            group = [r for r in rows if r["density"] == density and r["controller"] == controller]
            rates.append(100.0 * sum(1 for r in group if r["success"]) / max(len(group), 1))
        ax.bar(np.arange(len(densities)) + offset, rates, width,
               label="A*+DWA" if controller == "astar_dwa" else "DWA")
    ax.set_xticks(np.arange(len(densities)))
    ax.set_xticklabels([str(d) for d in densities])
    ax.set_xlabel("obstacles per world")
    ax.set_ylabel("success rate [%]")
    ax.set_ylim(0, 105)
    ax.grid(axis="y", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(figure_dir, "success_vs_density.png"), dpi=150)
    plt.close(fig)


def _plot_clearance(rows, figure_dir: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    densities = sorted({r["density"] for r in rows})
    data, labels, colors = [], [], []
    for density in densities:
        for controller, color in (("astar_dwa", "#1b9e77"), ("dwa", "#d95f02")):
            values = [r["min_clearance_m"] for r in rows
                      if r["density"] == density and r["controller"] == controller]
            if values:
                data.append(values)
                labels.append(f"{density}\n{'A*+DWA' if controller == 'astar_dwa' else 'DWA'}")
                colors.append(color)
    fig, ax = plt.subplots(figsize=(8.4, 4.2))
    box = ax.boxplot(data, patch_artist=True, tick_labels=labels)
    for patch, color in zip(box["boxes"], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.65)
    ax.axhline(0.30, color="#444444", linestyle="--", linewidth=1.0,
               label="planner safety margin (0.30 m)")
    ax.set_ylabel("minimum clearance [m] (all runs)")
    ax.grid(axis="y", alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(figure_dir, "clearance_box.png"), dpi=150)
    plt.close(fig)


def _plot_comfort(rows, summaries, figure_dir: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(11.0, 4.0))
    for controller, color in (("astar_dwa", "#1b9e77"), ("dwa", "#d95f02")):
        subset = [s for s in summaries if s["controller"] == controller and s["mean_acc_rms"]]
        if not subset:
            continue
        axes[0].plot([s["density"] for s in subset], [s["mean_acc_rms"] for s in subset],
                     "o-", color=color,
                     label="A*+DWA" if controller == "astar_dwa" else "DWA")
        axes[1].plot([s["density"] for s in subset], [s["mean_jerk_rms"] for s in subset],
                     "o-", color=color,
                     label="A*+DWA" if controller == "astar_dwa" else "DWA")
    for ax, title in zip(axes, ("linear acceleration RMS", "jerk RMS")):
        ax.set_xlabel("obstacles per world")
        ax.set_ylabel(title)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(figure_dir, "comfort.png"), dpi=150)
    plt.close(fig)




