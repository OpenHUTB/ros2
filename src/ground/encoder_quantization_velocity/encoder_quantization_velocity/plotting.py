"""Figures for the encoder quantisation report."""

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


def plot_velocity(profile, summary, output_path) -> None:
    if not HAVE_MATPLOTLIB:
        raise RuntimeError("matplotlib is required for figures")
    _style()
    from .core import alpha_beta, kalman_velocity, moving_average, raw_difference
    from .scene import measure

    data = measure(profile, 4096)
    dt = data["dt"]
    raw = raw_difference(data["position_rad"], dt)
    smooth = moving_average(raw, 5)
    kf = kalman_velocity(data["position_rad"], dt, process_sigma=25.0,
                         measurement_sigma=data["step_rad"] / 3.0)
    time_s = profile["time_s"]
    figure, axes = plt.subplots(2, 1, figsize=(7.2, 4.8), sharex=True)
    axes[0].plot(time_s, profile["velocity_rad_s"], color="#111111", lw=1.6, label="true")
    axes[0].plot(time_s, raw, color="#c0392b", lw=0.6, alpha=0.7, label="raw difference")
    axes[0].plot(time_s, kf, color="#1e8449", lw=1.2, label="Kalman")
    axes[0].set_ylabel("velocity [rad/s]")
    axes[0].set_title("4096 CPR encoder, 1 kHz")
    axes[0].legend(frameon=False, fontsize=8)
    axes[1].plot(time_s, raw - profile["velocity_rad_s"], color="#c0392b", lw=0.6,
                 alpha=0.7, label="raw")
    axes[1].plot(time_s, smooth - profile["velocity_rad_s"], color="#ef8a3c", lw=0.8,
                 label="moving average (5)")
    axes[1].plot(time_s, kf - profile["velocity_rad_s"], color="#1e8449", lw=1.0,
                 label="Kalman")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("error [rad/s]")
    axes[1].legend(frameon=False, fontsize=8)
    figure.tight_layout()
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)


def plot_resolution(summary, output_path) -> None:
    if not HAVE_MATPLOTLIB:
        raise RuntimeError("matplotlib is required for figures")
    _style()
    resolutions = [int(key) for key in summary["resolutions"]]
    figure, ax = plt.subplots(figsize=(6.4, 3.8))
    for name, color in (("raw", "#c0392b"), ("moving_average", "#ef8a3c"),
                        ("alpha_beta", "#2c6fbb"), ("kalman", "#1e8449")):
        errors = [summary["resolutions"][str(value)][name]["rmse_rad_s"] for value in resolutions]
        ax.loglog(resolutions, errors, "o-", color=color, label=name)
    ax.set_xlabel("encoder resolution [CPR = 4 x lines]")
    ax.set_ylabel("velocity RMSE [rad/s]")
    ax.set_title("Estimation error versus encoder resolution")
    ax.legend(frameon=False, fontsize=8)
    figure.tight_layout()
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)
