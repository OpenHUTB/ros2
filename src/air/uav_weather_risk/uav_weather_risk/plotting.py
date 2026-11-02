"""Headless figures generated from the synthetic demo records."""

from __future__ import annotations

import os
from typing import Sequence

RISK_COLORS = ("#2e7d32", "#f9a825", "#ef6c00", "#c62828")


def _pyplot():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt


def _prepare(path: str) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)


def plot_trajectory(records: Sequence[dict], path: str) -> None:
    plt = _pyplot()
    _prepare(path)
    fig, ax = plt.subplots(figsize=(7.2, 5.6), dpi=140)
    ax.plot(
        [r["target_x"] for r in records],
        [r["target_y"] for r in records],
        color="#90a4ae",
        linestyle="--",
        linewidth=1.0,
        label="waypoint sequence",
    )
    ax.plot([r["x"] for r in records], [r["y"] for r in records], color="#263238", linewidth=1.0)
    for level, name in enumerate(("NOMINAL", "CAUTION", "WARNING", "CRITICAL")):
        points = [r for r in records if r["risk_level"] == level]
        if points:
            ax.scatter(
                [r["x"] for r in points],
                [r["y"] for r in points],
                s=18,
                color=RISK_COLORS[level],
                label=name,
                zorder=3,
            )
    for record in records[::20]:
        ax.arrow(
            record["x"],
            record["y"],
            record["true_wind_x"] * 0.35,
            record["true_wind_y"] * 0.35,
            color="#1565c0",
            width=0.03,
            head_width=0.25,
            length_includes_head=True,
            alpha=0.7,
        )
    ax.scatter(
        [records[0]["x"]],
        [records[0]["y"]],
        marker="o",
        s=70,
        facecolors="white",
        edgecolors="#263238",
        label="start",
        zorder=4,
    )
    ax.set_xlabel("east [m]")
    ax.set_ylabel("north [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.25)
    ax.legend(loc="best", fontsize=8, frameon=False)
    ax.set_title("Weather-induced trajectory and risk level")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def plot_risk(records: Sequence[dict], path: str) -> None:
    plt = _pyplot()
    _prepare(path)
    time = [r["t"] for r in records]
    fig, axes = plt.subplots(3, 1, figsize=(8.2, 7.4), dpi=140, sharex=True)
    axes[0].plot(time, [r["true_wind_speed"] for r in records], color="#1565c0", label="true wind")
    axes[0].plot(time, [r["fused_wind_speed"] for r in records], color="#ef6c00", label="fused estimate")
    axes[0].axhline(11.0, color="#c62828", linestyle=":", linewidth=1.0, label="11 m/s threshold")
    axes[0].set_ylabel("wind speed [m/s]")
    axes[0].legend(loc="upper left", fontsize=8, frameon=False)

    for key, label, color in (
        ("p_tilt", "tilt exceedance", "#6a1b9a"),
        ("p_position", "trajectory deviation", "#00838f"),
        ("p_control", "control saturation", "#5d4037"),
    ):
        axes[1].plot(time, [r[key] for r in records], label=label, color=color)
    axes[1].set_ylim(-0.03, 1.03)
    axes[1].set_ylabel("probability")
    axes[1].legend(loc="upper left", fontsize=8, frameon=False)

    axes[2].plot(time, [r["risk_score"] for r in records], color="#c62828", label="risk score")
    for level, name in enumerate(("NOMINAL", "CAUTION", "WARNING", "CRITICAL")):
        axes[2].axhspan(
            (0.0, 0.2, 0.45, 0.7)[level],
            (0.2, 0.45, 0.7, 1.0)[level],
            color=RISK_COLORS[level],
            alpha=0.08,
            label=name,
        )
    axes[2].set_ylim(0.0, 1.03)
    axes[2].set_ylabel("weighted score")
    axes[2].set_xlabel("simulation time [s]")
    axes[2].legend(loc="upper left", fontsize=8, frameon=False)
    for ax in axes:
        ax.grid(alpha=0.22)
    fig.suptitle("Short-horizon disturbance prediction and safety assessment")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def plot_sources(records: Sequence[dict], path: str) -> None:
    plt = _pyplot()
    _prepare(path)
    time = [r["t"] for r in records]
    fig, ax = plt.subplots(figsize=(8.0, 4.2), dpi=140)
    for key, label, color in (
        ("nwp_speed", "NWP background", "#7b1fa2"),
        ("ground_speed", "ground mast", "#2e7d32"),
        ("onboard_speed", "onboard estimate", "#1565c0"),
        ("fused_wind_speed", "fused", "#ef6c00"),
    ):
        ax.plot(time, [r[key] for r in records], label=label, color=color)
    ax.set_xlabel("simulation time [s]")
    ax.set_ylabel("wind speed [m/s]")
    ax.grid(alpha=0.22)
    ax.legend(loc="upper left", fontsize=8, frameon=False)
    ax.set_title("Multi-source wind fusion")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def animate_demo(records: Sequence[dict], path: str, *, frame_stride: int = 10) -> bool:
    """Create a top-down GIF; return False when Pillow is unavailable."""

    try:
        from matplotlib.animation import FuncAnimation, PillowWriter
    except ImportError:
        return False
    plt = _pyplot()
    _prepare(path)
    frames = records[:: max(1, frame_stride)]
    if len(frames) < 2:
        return False
    x_values = [r["x"] for r in records]
    y_values = [r["y"] for r in records]
    margin = 3.0
    fig, ax = plt.subplots(figsize=(7.2, 4.8), dpi=100)
    ax.set_xlim(min(x_values) - margin, max(x_values) + margin)
    ax.set_ylim(min(y_values) - margin, max(y_values) + margin)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("east [m]")
    ax.set_ylabel("north [m]")
    ax.grid(alpha=0.2)
    ax.plot(
        [r["target_x"] for r in records],
        [r["target_y"] for r in records],
        color="#90a4ae",
        linestyle="--",
        linewidth=1.0,
    )
    line, = ax.plot([], [], color="#263238", linewidth=1.2)
    point = ax.scatter([], [], s=50, zorder=3)
    arrow = ax.arrow(0, 0, 0, 0, color="#1565c0", width=0.05, head_width=0.3)
    label = ax.text(0.02, 0.96, "", transform=ax.transAxes, va="top", fontsize=10)

    def update(index):
        nonlocal arrow
        frame = frames[index]
        visible = [r for r in records if r["t"] <= frame["t"]]
        line.set_data([r["x"] for r in visible], [r["y"] for r in visible])
        point.set_offsets([[frame["x"], frame["y"]]])
        point.set_color(RISK_COLORS[frame["risk_level"]])
        arrow.remove()
        arrow = ax.arrow(
            frame["x"],
            frame["y"],
            frame["true_wind_x"] * 0.35,
            frame["true_wind_y"] * 0.35,
            color="#1565c0",
            width=0.04,
            head_width=0.28,
            length_includes_head=True,
        )
        label.set_text(
            f"t={frame['t']:5.1f} s   {frame['risk_name']}\n"
            f"wind={frame['fused_wind_speed']:.1f} m/s   score={frame['risk_score']:.2f}"
        )
        return line, point, arrow, label

    animation = FuncAnimation(fig, update, frames=len(frames), interval=120, blit=False)
    animation.save(path, writer=PillowWriter(fps=8))
    plt.close(fig)
    return True
