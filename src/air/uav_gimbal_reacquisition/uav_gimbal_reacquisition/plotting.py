"""Matplotlib figures and a top-down GIF for the gimbal demo."""

from __future__ import annotations

import math
import os

RISK_COLORS = {"SEARCH": "#ffb300", "TRACK": "#2e7d32", "LOST": "#c62828", "REACQUIRE": "#1565c0"}


def _pyplot():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt


def _dir(path):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)


def plot_state_timeline(records, path):
    plt = _pyplot()
    _dir(path)
    t = [r["t"] for r in records]
    fig, axes = plt.subplots(3, 1, figsize=(9, 7.5), dpi=140, sharex=True)
    axes[0].plot(t, [r["target_yaw_deg"] for r in records], color="#1565c0", label="target bearing yaw")
    axes[0].plot(t, [r["yaw_deg"] for r in records], color="#ef6c00", label="gimbal yaw")
    axes[0].set_ylabel("yaw [deg]")
    axes[0].legend(fontsize=8, frameon=False, loc="best")
    axes[1].plot(t, [r["target_pitch_deg"] for r in records], color="#1565c0", label="target bearing pitch")
    axes[1].plot(t, [r["pitch_deg"] for r in records], color="#ef6c00", label="gimbal pitch")
    axes[1].set_ylabel("pitch [deg]")
    axes[1].legend(fontsize=8, frameon=False, loc="best")
    axes[2].plot(t, [r["los_error_deg"] for r in records], color="#c62828", label="line-of-sight error")
    axes[2].set_ylabel("LOS error [deg]")
    axes[2].set_xlabel("time [s]")
    axes[2].legend(fontsize=8, frameon=False, loc="best")
    for ax in axes:
        ax.grid(alpha=0.25)
    fig.suptitle("UAV gimbal tracking and line-of-sight error")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def plot_pixel_error(records, path):
    plt = _pyplot()
    _dir(path)
    t = [r["t"] for r in records]
    fig, ax = plt.subplots(figsize=(8.4, 4.2), dpi=140)
    ax.plot(t, [math.nan if r["pixel_u"] is None else r["pixel_u"] for r in records], color="#1565c0", label="pixel u")
    ax.plot(t, [math.nan if r["pixel_v"] is None else r["pixel_v"] for r in records], color="#ef6c00", label="pixel v")
    ax.axhline(320, color="#90a4ae", linestyle=":", linewidth=1)
    ax.axhline(-320, color="#90a4ae", linestyle=":", linewidth=1)
    ax.axhline(240, color="#cfd8dc", linestyle="--", linewidth=1)
    ax.axhline(-240, color="#cfd8dc", linestyle="--", linewidth=1)
    ax.set_ylabel("pixel error at 640x480 [px]")
    ax.set_xlabel("time [s]")
    ax.set_title("Image-plane error and FOV limits")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8, frameon=False, loc="best")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def plot_topdown(records, path):
    plt = _pyplot()
    _dir(path)
    fig, ax = plt.subplots(figsize=(7.2, 5.6), dpi=140)
    ax.plot([r["target_x"] for r in records], [r["target_y"] for r in records], color="#90a4ae", linewidth=1, label="target path")
    ax.scatter(0, 0, marker="^", s=100, color="#1565c0", label="UAV")
    for state, color in RISK_COLORS.items():
        points = [r for r in records if r["state"] == state]
        if points:
            ax.scatter([r["target_x"] for r in points], [r["target_y"] for r in points], s=8, color=color, label=state)
    ax.set_xlabel("east [m]")
    ax.set_ylabel("north [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8, frameon=False, loc="best")
    ax.set_title("Top-down target path and gimbal state")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def animate_demo(records, path, frame_stride=20):
    try:
        from matplotlib.animation import FuncAnimation, PillowWriter
    except ImportError:
        return False
    plt = _pyplot()
    _dir(path)
    frames = records[:: max(1, frame_stride)]
    if len(frames) < 2:
        return False
    fig, ax = plt.subplots(figsize=(8, 5), dpi=100)
    ax.set_xlim(-20, 55)
    ax.set_ylim(-25, 40)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("east [m]")
    ax.set_ylabel("north [m]")
    ax.grid(alpha=0.2)
    ax.scatter([0], [0], marker="^", s=100, color="#1565c0", label="UAV")
    target_dot = ax.scatter([], [], s=60, zorder=4)
    path_line, = ax.plot([], [], color="#546e7a", linewidth=1)
    boresight, = ax.plot([], [], color="#ef6c00", linewidth=1.5)
    label = ax.text(0.02, 0.96, "", transform=ax.transAxes, va="top", fontsize=10)
    ax.legend(fontsize=8, frameon=False, loc="lower left")

    def update(index):
        frame = frames[index]
        visible = [r for r in records if r["t"] <= frame["t"]]
        path_line.set_data([r["target_x"] for r in visible], [r["target_y"] for r in visible])
        target_dot.set_offsets([[frame["target_x"], frame["target_y"]]])
        target_dot.set_color("#2e7d32" if frame["detected"] else "#c62828")
        yaw = math.radians(frame["yaw_deg"])
        boresight.set_data([0, 35 * math.cos(yaw)], [0, 35 * math.sin(yaw)])
        label.set_text(
            f"t={frame['t']:4.1f}s  {frame['state']}\n"
            f"LOS error={frame['los_error_deg']:5.1f} deg"
        )
        return path_line, target_dot, boresight, label

    animation = FuncAnimation(fig, update, frames=len(frames), interval=120, blit=False)
    animation.save(path, writer=PillowWriter(fps=8))
    plt.close(fig)
    return True
