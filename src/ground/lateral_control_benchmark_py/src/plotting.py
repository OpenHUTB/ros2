"""Matplotlib 绘图。图中所有坐标轴标签和标题均使用英文。"""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

CONTROLLER_COLORS = {
    "pure_pursuit": "#1f77b4",
    "stanley": "#d95f02",
    "lqr": "#2ca02c",
}
CONTROLLER_LABELS = {
    "pure_pursuit": "Pure Pursuit",
    "stanley": "Stanley",
    "lqr": "LQR",
}


def _style():
    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 9.5,
        "axes.titlesize": 11,
        "axes.labelsize": 10,
        "legend.fontsize": 8.5,
        "axes.grid": True,
        "grid.alpha": 0.25,
        "figure.dpi": 120,
        "savefig.bbox": "tight",
    })


def plot_metrics(records, output_path):
    """按场景分组展示关键指标。"""
    _style()
    metrics = [
        ("mean_abs", "Mean absolute lateral error [m]"),
        ("rms", "RMS lateral error [m]"),
        ("max_abs", "Maximum absolute lateral error [m]"),
        ("steady_abs", "Steady-state |e| (last 25%) [m]"),
        ("convergence_s", "Convergence time [s]"),
        ("steer_rate_rms", "Steering rate RMS [rad/s]"),
    ]
    scenarios = []
    for item in records:
        if item["scenario"] not in scenarios:
            scenarios.append(item["scenario"])
    controllers = []
    for item in records:
        if item["controller"] not in controllers:
            controllers.append(item["controller"])

    short_labels = {
        "straight": "Straight",
        "arc": "Arc\nR=20 m",
        "sine": "Sine\nA=3 m",
        "double_lane": "ISO 3888\nDouble lane",
    }
    fig, axes = plt.subplots(2, 3, figsize=(13.2, 7.4))
    x = np.arange(len(scenarios), dtype=float)
    width = 0.24
    for ax, (key, title) in zip(axes.ravel(), metrics):
        for j, controller in enumerate(controllers):
            values = []
            for scenario in scenarios:
                match = next(r for r in records
                             if r["scenario"] == scenario and r["controller"] == controller)
                values.append(match[key])
            ax.bar(x + (j - 1) * width, values, width,
                   color=CONTROLLER_COLORS.get(controller, None),
                   label=CONTROLLER_LABELS.get(controller, controller), edgecolor="white")
        labels = [short_labels.get(scenario, scenario.replace("_", " ").title())
                  for scenario in scenarios]
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=0)
        ax.set_title(title)
        ax.set_ylabel("Value")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=3, frameon=False,
               bbox_to_anchor=(0.5, 1.02))
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(output_path, dpi=170)
    plt.close(fig)


def plot_arc_trajectories(results, scenario, output_path):
    """圆弧场景的轨迹对比。"""
    _style()
    fig, ax = plt.subplots(figsize=(7.2, 6.3))
    path = scenario.path
    ax.plot(path.x, path.y, "k--", linewidth=1.5, label="Reference path")
    for result in results:
        ax.plot(result.x, result.y, color=CONTROLLER_COLORS.get(result.controller, None),
                linewidth=1.6, label=result.label)
        ax.scatter([result.x[0]], [result.y[0]], s=26,
                   color=CONTROLLER_COLORS.get(result.controller, None), zorder=5)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Arc R=20 m: Vehicle Trajectories")
    ax.legend(loc="best", frameon=True)
    fig.tight_layout()
    fig.savefig(output_path, dpi=170)
    plt.close(fig)


def plot_error_curves(results, scenario, output_path):
    """指定场景的横向误差时间历程。"""
    _style()
    fig, ax = plt.subplots(figsize=(9.2, 4.6))
    for result in results:
        ax.plot(result.time, result.e, color=CONTROLLER_COLORS.get(result.controller, None),
                linewidth=1.7, label=result.label)
    ax.axhline(0.0, color="black", linewidth=0.8, alpha=0.6)
    ax.axhline(0.2, color="gray", linestyle=":", linewidth=1.0)
    ax.axhline(-0.2, color="gray", linestyle=":", linewidth=1.0)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Lateral error e [m] (left positive)")
    ax.set_title(f"{scenario.title}: Lateral Tracking Error")
    ax.legend(loc="best", ncol=3, frameon=True)
    fig.tight_layout()
    fig.savefig(output_path, dpi=170)
    plt.close(fig)


def plot_single(scenario, results, trajectory_path, error_path):
    """单场景：轨迹 + 误差，两张图。"""
    _style()
    fig, ax = plt.subplots(figsize=(8.6, 4.9))
    path = scenario.path
    ax.plot(path.x, path.y, "k--", linewidth=1.4, label="Reference path")
    for result in results:
        ax.plot(result.x, result.y, color=CONTROLLER_COLORS.get(result.controller, None),
                linewidth=1.6, label=result.label)
    if scenario.key == "arc":
        ax.set_aspect("equal", adjustable="box")
    else:
        # 长距离场景保持可读性，横向比例适当放大。
        ax.set_aspect("auto")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"{scenario.title}: Closed-loop Trajectories")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(trajectory_path, dpi=170)
    plt.close(fig)

    plot_error_curves(results, scenario, error_path)


def plot_sweep(rows, output_path):
    """速度扫描：平均绝对误差和收敛时间。"""
    _style()
    controllers = []
    for row in rows:
        if row["controller"] not in controllers:
            controllers.append(row["controller"])
    fig, axes = plt.subplots(1, 2, figsize=(11.0, 4.4))
    for controller in controllers:
        subset = sorted([r for r in rows if r["controller"] == controller],
                        key=lambda r: r["speed"])
        speeds = [r["speed"] for r in subset]
        color = CONTROLLER_COLORS.get(controller, None)
        label = CONTROLLER_LABELS.get(controller, controller)
        axes[0].plot(speeds, [r["mean_abs"] for r in subset], "o-", color=color, label=label)
        axes[1].plot(speeds, [r["convergence_s"] for r in subset], "o-", color=color, label=label)
    axes[0].set_xlabel("Cruise speed [m/s]")
    axes[0].set_ylabel("Mean |e| across scenarios [m]")
    axes[0].set_title("Tracking Error vs Speed")
    axes[1].set_xlabel("Cruise speed [m/s]")
    axes[1].set_ylabel("Convergence time [s]")
    axes[1].set_title("Convergence vs Speed")
    axes[1].legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_path, dpi=170)
    plt.close(fig)
