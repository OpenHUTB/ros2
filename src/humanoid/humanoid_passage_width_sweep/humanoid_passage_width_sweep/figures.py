# -*- coding: utf-8 -*-
"""图表：通过率-门宽、最小净空-门宽、轨迹对比、演示动图。"""

from pathlib import Path

import numpy as np

import humanoid_passage_width_sweep.config as config

_COLORS = {
    "mppi_footprint": "#1f77b4",
    "mppi_point": "#d95f02",
    "reactive_footprint": "#2ca02c",
    "reactive_point": "#9467bd",
}
_LABEL_EN = {
    "mppi_footprint": "MPPI (footprint-aware)",
    "mppi_point": "MPPI (point-mass)",
    "reactive_footprint": "reactive (footprint-aware)",
    "reactive_point": "reactive (point-mass)",
}

_MARKERS = {
    "mppi_footprint": "o", "mppi_point": "s",
    "reactive_footprint": "^", "reactive_point": "v",
}


def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "figure.dpi": 130, "font.size": 9,
        "axes.spines.top": False, "axes.spines.right": False,
        "axes.grid": True, "grid.alpha": 0.25, "grid.linestyle": "--",
    })
    return plt


def plot_success(agg, path: Path) -> Path:
    """通过率 vs 门宽（每档两条：成功/总次数）。"""
    plt = _plt()
    fig, ax = plt.subplots(figsize=(6.2, 3.2))
    for v in {r["variant"] for r in agg}:
        rows = sorted([r for r in agg if r["variant"] == v], key=lambda r: r["width"])
        ax.plot([r["width"] for r in rows], [r["success_rate"] * 100 for r in rows],
                marker=_MARKERS.get(v, "o"), color=_COLORS.get(v), lw=1.4, ms=4,
                label=_LABEL_EN.get(v, v))
    ax.axvline(2 * config.FOOTPRINT_RADIUS, color="0.35", ls=":", lw=1.0)
    ax.text(2 * config.FOOTPRINT_RADIUS + 0.01, 50, "footprint diameter\n0.60 m",
            fontsize=7, color="0.35", va="center")
    ax.set_xlabel("gate clear width (m)")
    ax.set_ylabel("success rate (%)")
    ax.set_ylim(-5, 105)
    ax.set_title("Passage success vs gate width (2 runs each)")
    ax.legend(frameon=False, fontsize=7.5, loc="lower right")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)
    return path


def plot_clearance(agg, path: Path) -> Path:
    """最小净空 vs 门宽；0 表示刮蹭。"""
    plt = _plt()
    fig, ax = plt.subplots(figsize=(6.2, 3.0))
    for v in {r["variant"] for r in agg}:
        rows = sorted([r for r in agg if r["variant"] == v], key=lambda r: r["width"])
        ax.plot([r["width"] for r in rows], [r["min_clear"] for r in rows],
                marker=_MARKERS.get(v, "o"), color=_COLORS.get(v), lw=1.4, ms=4,
                label=_LABEL_EN.get(v, v))
    ax.axhline(0.0, color="#c1121f", lw=1.0, ls="--")
    ax.set_xlabel("gate clear width (m)")
    ax.set_ylabel("min clearance (m)")
    ax.set_title("Minimum clearance during the run (negative = contact)")
    ax.legend(frameon=False, fontsize=7.5, loc="lower right")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return path


def plot_trajectory(result, path: Path) -> Path:
    """一档门宽下的 x-y 轨迹（俯视），画出走廊、窄门与障碍。"""
    plt = _plt()
    log = result.log
    fig, ax = plt.subplots(figsize=(7.4, 3.0))
    side = config.CORRIDOR_HALF_WIDTH
    ax.plot([0, 9], [side, side], color="0.35", lw=2.0)
    ax.plot([0, 9], [-side, -side], color="0.35", lw=2.0)
    half = result.width / 2
    ax.plot([config.GATE_X, config.GATE_X], [half, side], color="0.35", lw=3.0)
    ax.plot([config.GATE_X, config.GATE_X], [-side, -half], color="0.35", lw=3.0)
    for (ox, oy) in config.OBSTACLES:
        ax.add_patch(plt.Circle((ox, oy), config.OBSTACLE_RADIUS,
                                color="#e08b2a", alpha=0.85))
    if len(log.get("x", [])):
        ax.plot(log["x"], log["y"], lw=1.5,
                color=_COLORS.get(result.variant, "#1f77b4"),
                label=_LABEL_EN.get(result.variant, result.variant))
    ax.set_aspect("equal")
    ax.set_xlim(-0.3, 9.0)
    ax.set_ylim(-1.2, 1.2)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"Trajectory at gate width {result.width:.2f} m")
    ax.legend(frameon=False, fontsize=8, loc="lower right")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)
    return path


def save_snapshot(sim, path):
    """保存一帧场景快照（单帧渲染是可靠的；多帧 GIF 在本模块踩过坑，不做）。"""
    from PIL import Image
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(sim.render(640, 360, "overview")).save(path)
    return path


def snapshots(variant="mppi_footprint", width=config.DEFAULT_WIDTH, outdir="results"):
    """跑一趟，在三个关键时刻各存一张图：出发、接近障碍、过门瞬间。"""
    from humanoid_passage_width_sweep.sim import PassageSim
    outdir = Path(outdir)
    sim = PassageSim(width=width, variant=variant)
    saved = {}
    marks = {"start": 0.0, "obstacle": 2.05, "gate": 3.75, "done": 7.6}
    for _ in range(int(config.TIME_LIMIT * config.CONTROL_HZ)):
        sim.step()
        x = float(sim.data.qpos[sim._jx])
        for key, tx in list(marks.items()):
            if key not in saved and x >= tx:
                saved[key] = save_snapshot(sim, outdir / f"scene_{key}.png")
        if sim.goal_distance() <= config.GOAL_RADIUS or sim.in_contact():
            break
    sim.close()
    return saved
