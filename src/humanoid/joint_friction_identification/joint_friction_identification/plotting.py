"""Figures for the joint-friction report."""

from __future__ import annotations

import numpy as np

try:  # pragma: no cover
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    HAVE_MATPLOTLIB = True
except Exception:  # pragma: no cover
    HAVE_MATPLOTLIB = False


def _style() -> None:
    plt.rcParams.update(
        {"figure.dpi": 150, "font.size": 9, "axes.grid": True, "grid.alpha": 0.3}
    )


def plot_friction(sweeps, model, output_path) -> None:
    if not HAVE_MATPLOTLIB:
        raise RuntimeError("matplotlib is required for figures")
    _style()
    speeds = np.asarray(sweeps["speeds"])
    torques = np.asarray(sweeps["torques"])
    grid = np.linspace(0.0, 1.3, 300)
    figure, ax = plt.subplots(figsize=(6.4, 4.0))
    ax.plot(grid, model.torque(grid), "-", color="#1e8449", lw=1.6, label="identified model")
    ax.plot(grid, model.torque(-grid), "-", color="#1e8449", lw=1.6)
    ax.plot(speeds, torques, "o", color="#c0392b", ms=5, label="sweep measurements")
    ax.plot(-speeds, -torques, "o", color="#c0392b", ms=5)
    ax.set_xlabel("joint velocity [rad/s]")
    ax.set_ylabel("friction torque [N m]")
    ax.set_title("Stribeck friction identified from velocity sweeps")
    ax.legend(frameon=False, fontsize=8)
    figure.tight_layout()
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)


def plot_tracking(baseline, compensated, output_path) -> None:
    if not HAVE_MATPLOTLIB:
        raise RuntimeError("matplotlib is required for figures")
    _style()
    time_s = baseline["time_s"]
    figure, axes = plt.subplots(2, 1, figsize=(7.2, 4.8), sharex=True)
    axes[0].plot(time_s, baseline["error"], color="#c0392b", lw=0.8, label="PD only")
    axes[0].plot(time_s, compensated["error"], color="#1e8449", lw=0.8,
                 label="PD + friction feed-forward")
    axes[0].set_ylabel("tracking error [rad]")
    axes[0].set_title("Sine tracking with PD control")
    axes[0].legend(frameon=False, fontsize=8)
    axes[1].plot(time_s, baseline["torque"], color="#c0392b", lw=0.7, alpha=0.7,
                 label="PD only")
    axes[1].plot(time_s, compensated["torque"], color="#1e8449", lw=0.7,
                 label="compensated")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("commanded torque [N m]")
    axes[1].legend(frameon=False, fontsize=8)
    figure.tight_layout()
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)
