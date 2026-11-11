"""matplotlib 绘图。图中所有坐标轴、图例和标题均使用英文。"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .footstep import FootstepPlan
from .gait import GaitResult, result_to_row

plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "axes.grid": True,
        "grid.alpha": 0.25,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "figure.facecolor": "white",
    }
)

BLUE = "#2b6cb0"
ORANGE = "#dd6b20"
GRAY = "#718096"
GREEN = "#2f855a"
RED = "#c53030"


def _foot_blocks(ax: plt.Axes, plan: FootstepPlan, y: float = -0.13) -> None:
    for step in plan.steps:
        color = BLUE if step.side == "L" else ORANGE
        ax.fill_between(
            [step.land_time, step.lift_time],
            y - 0.018,
            y + 0.018,
            color=color,
            alpha=0.55,
            linewidth=0.0,
        )


def plot_gait(result: GaitResult, plan: FootstepPlan, path: str | Path) -> None:
    fig, ax = plt.subplots(figsize=(11.5, 5.8), constrained_layout=True)
    ax.fill_between(
        result.time,
        result.support_left,
        result.support_right,
        color="#cbd5e0",
        alpha=0.45,
        linewidth=0.0,
        label="ZMP support interval",
    )
    ax.plot(result.time, result.x, color=BLUE, lw=2.2, label="COM x")
    ax.plot(
        result.time,
        result.zmp_ref,
        color=GRAY,
        lw=1.5,
        ls="--",
        label="ZMP reference",
    )
    ax.plot(result.time, result.zmp, color=ORANGE, lw=1.4, label="ZMP applied")
    _foot_blocks(ax, plan)
    ax.axhline(0.0, color="black", lw=0.8, alpha=0.5)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("X position [m]")
    ax.set_title(
        f"Bipedal gait: COM and ZMP  (K = {result.gain:.1f}, "
        f"travel = {result.travel:.2f} m)"
    )
    ax.set_xlim(result.time[0], result.time[-1])
    ax.legend(loc="upper left", ncol=2, frameon=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_stability_margin(result: GaitResult, path: str | Path) -> None:
    fig, ax = plt.subplots(figsize=(11.5, 4.2), constrained_layout=True)
    ax.fill_between(
        result.time,
        0.0,
        result.margin,
        color=GREEN,
        alpha=0.18,
        linewidth=0.0,
    )
    ax.plot(result.time, result.margin, color=GREEN, lw=1.6)
    ax.axhline(0.0, color=RED, lw=1.0, ls="--")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Margin to support boundary [m]")
    ax.set_title(
        f"ZMP stability margin  (mean = {result.mean_margin:.4f} m, "
        f"min = {result.min_margin:.4f} m)"
    )
    ax.set_xlim(result.time[0], result.time[-1])
    ax.set_ylim(bottom=-0.005)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_dcm_error(result: GaitResult, path: str | Path) -> None:
    huge = float(np.max(np.abs(result.dcm))) > 100.0
    fig, axes = plt.subplots(
        2,
        1,
        figsize=(11.5, 6.0),
        sharex=True,
        constrained_layout=True,
    )
    top, bottom = axes
    top.plot(result.time, result.dcm_ref, color=GRAY, lw=1.5, ls="--", label="DCM reference")
    top.plot(result.time, result.dcm, color=BLUE, lw=1.7, label="DCM actual")
    top.set_ylabel("DCM [m]" + ("  (symlog)" if huge else ""))
    top.set_title(f"Divergent component of motion, K = {result.gain:.1f}")
    top.legend(loc="upper left")

    bottom.plot(result.time, result.dcm_error, color=ORANGE, lw=1.4)
    bottom.set_xlabel("Time [s]")
    bottom.set_ylabel("|DCM error| [m]" + ("  (symlog)" if huge else ""))
    bottom.set_title(f"Maximum DCM error = {result.dcm_max_error:.4f} m")

    if huge:
        top.set_yscale("symlog", linthresh=1.0)
        bottom.set_yscale("symlog", linthresh=1.0)
    bottom.set_xlim(result.time[0], result.time[-1])
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_gain_sweep(
    results: list[GaitResult],
    current_gain: float,
    path: str | Path,
) -> None:
    rows = [result_to_row(r) for r in results]
    gains = np.array([row["gain"] for row in rows], dtype=float)
    travel = np.array([row["travel_m"] for row in rows], dtype=float)
    saturation = 100.0 * np.array([row["saturation_ratio"] for row in rows], dtype=float)
    margin = np.array([row["mean_margin_m"] for row in rows], dtype=float)
    stable = np.array([row["stable"] for row in rows], dtype=bool)

    fig, axes = plt.subplots(
        3,
        1,
        figsize=(10.5, 8.2),
        sharex=True,
        constrained_layout=True,
    )
    ax0, ax1, ax2 = axes

    ax0.plot(gains, travel, color=GRAY, lw=1.0, zorder=1)
    ax0.scatter(gains[stable], travel[stable], s=64, color=GREEN, zorder=3, label="walking")
    if np.any(~stable):
        ax0.scatter(
            gains[~stable],
            travel[~stable],
            s=64,
            color=RED,
            marker="x",
            zorder=3,
            label="diverged",
        )
    ax0.set_yscale("symlog", linthresh=10.0)
    ax0.set_ylabel("COM travel [m]  (symlog)")
    ax0.set_title("DCM gain sweep: travel, ZMP saturation and mean stability margin")
    ax0.legend(loc="best")
    ax0.axvline(current_gain, color=BLUE, ls=":", lw=1.2)

    ax1.plot(gains, saturation, color=ORANGE, marker="o", lw=1.5)
    ax1.set_ylabel("ZMP saturation [%]")
    ax1.set_ylim(0, 100)
    ax1.axvline(current_gain, color=BLUE, ls=":", lw=1.2)

    ax2.plot(gains, margin, color=GREEN, marker="o", lw=1.5)
    ax2.set_xlabel("DCM feedback gain K")
    ax2.set_ylabel("Mean margin [m]")
    ax2.axvline(current_gain, color=BLUE, ls=":", lw=1.2, label=f"selected K={current_gain:.1f}")
    ax2.legend(loc="best")
    ax2.set_xlim(min(0.0, float(gains.min())), max(3.0, float(gains.max())))

    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_support_margin_sweep(
    rows: list[dict],
    gain: float,
    path: str | Path,
) -> None:
    """踝部边界缓冲 -> 行程 / 饱和比例 / 是否稳定。"""
    margin = np.array([row["zmp_edge_margin_m"] for row in rows], dtype=float)
    width = np.array([row["effective_half_width_m"] for row in rows], dtype=float)
    travel = np.array([row["travel_m"] for row in rows], dtype=float)
    saturation = 100.0 * np.array([row["saturation_ratio"] for row in rows], dtype=float)
    stable = np.array([row["stable"] for row in rows], dtype=bool)

    fig, axes = plt.subplots(2, 1, figsize=(9.5, 6.4), sharex=True,
                             constrained_layout=True)
    ax0, ax1 = axes

    ax0.plot(margin, travel, color=GRAY, lw=1.0, zorder=1)
    ax0.scatter(margin[stable], travel[stable], s=70, color=GREEN,
                zorder=3, label="walking")
    if np.any(~stable):
        ax0.scatter(margin[~stable], travel[~stable], s=70, color=RED,
                    marker="x", zorder=3, label="diverged")
    ax0.set_ylabel("COM travel [m]  (symlog)")
    ax0.set_yscale("symlog", linthresh=1.0)
    ax0.set_title(
        f"Ankle edge buffer vs. stability  (K = {gain:.1f}, "
        f"foot half-length = 0.09 m)"
    )
    ax0.legend(loc="best")

    ax1.plot(margin, saturation, color=ORANGE, marker="o", lw=1.5)
    ax1.set_xlabel("Ankle edge buffer [m]  (effective half-width = 0.09 + buffer)")
    ax1.set_ylabel("ZMP saturation [%]")
    ax1.set_ylim(0, 100)
    ax1.set_xticks(margin)
    ax1.set_xticklabels([f"{m:.2f}\n({w:.2f})" for m, w in zip(margin, width)])

    fig.savefig(path, dpi=150)
    plt.close(fig)


def generate_all_plots(
    result: GaitResult,
    plan: FootstepPlan,
    sweep_results: list[GaitResult],
    out_dir: str | Path,
) -> dict[str, Path]:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    paths = {
        "gait": out / "gait_com_zmp.png",
        "margin": out / "stability_margin.png",
        "dcm": out / "dcm_error.png",
        "sweep": out / "gain_sweep.png",
    }
    plot_gait(result, plan, paths["gait"])
    plot_stability_margin(result, paths["margin"])
    plot_dcm_error(result, paths["dcm"])
    plot_gain_sweep(sweep_results, result.gain, paths["sweep"])
    return paths
