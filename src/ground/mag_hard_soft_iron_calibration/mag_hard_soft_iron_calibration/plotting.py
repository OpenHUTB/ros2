"""Figures for the magnetometer calibration report."""

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


def plot_sphere(raw, corrected, output_path) -> None:
    if not HAVE_MATPLOTLIB:
        raise RuntimeError("matplotlib is required for figures")
    _style()
    figure, axes = plt.subplots(1, 2, figsize=(8.4, 4.0), subplot_kw={"projection": "3d"})
    for ax, data, title, color in (
        (axes[0], raw, "raw measurements", "#c0392b"),
        (axes[1], corrected, "after ellipsoid calibration", "#1e8449"),
    ):
        ax.scatter(data[:, 0], data[:, 1], data[:, 2], s=3, color=color, alpha=0.5)
        limit = float(np.max(np.abs(data))) * 1.1
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        ax.set_zlim(-limit, limit)
        ax.set_title(title, fontsize=9)
        ax.set_xlabel("x [uT]")
        ax.set_ylabel("y [uT]")
        ax.set_zlabel("z [uT]")
    figure.tight_layout()
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)


def plot_heading(time_s, raw, calibrated, fused, truth, output_path) -> None:
    if not HAVE_MATPLOTLIB:
        raise RuntimeError("matplotlib is required for figures")
    _style()
    figure, axes = plt.subplots(2, 1, figsize=(7.0, 5.0), sharex=True)

    def error(estimate):
        return np.rad2deg(np.arctan2(np.sin(estimate - truth), np.cos(estimate - truth)))

    axes[0].plot(time_s, error(raw), color="#c0392b", lw=0.8, label="raw magnetometer")
    axes[0].plot(time_s, error(calibrated), color="#1e8449", lw=0.8,
                 label="ellipsoid calibrated")
    axes[0].set_ylabel("heading error [deg]")
    axes[0].set_title("Heading error, before and after calibration")
    axes[0].legend(frameon=False, fontsize=8)
    axes[1].plot(time_s, error(calibrated), color="#1e8449", lw=0.7, alpha=0.6,
                 label="magnetometer only")
    axes[1].plot(time_s, error(fused), color="#2c6fbb", lw=1.0, label="gyro + magnetometer")
    axes[1].set_ylabel("heading error [deg]")
    axes[1].set_xlabel("time [s]")
    axes[1].legend(frameon=False, fontsize=8)
    figure.tight_layout()
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)
