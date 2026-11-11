"""Figures for the Allan-variance report."""

from __future__ import annotations

import numpy as np

try:  # pragma: no cover
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    HAVE_MATPLOTLIB = True
except Exception:  # pragma: no cover
    HAVE_MATPLOTLIB = False

SECONDS_PER_HOUR = 3600.0


def _style() -> None:
    plt.rcParams.update(
        {
            "figure.dpi": 150,
            "font.size": 9,
            "axes.grid": True,
            "grid.alpha": 0.3,
            "axes.axisbelow": True,
        }
    )


def plot_allan(taus, adev, identified, truth, output_path) -> None:
    if not HAVE_MATPLOTLIB:
        raise RuntimeError("matplotlib is required for figures")
    _style()
    figure, ax = plt.subplots(figsize=(6.4, 4.2))
    ax.loglog(taus, adev, "o", ms=3, color="#2c6fbb", label="measured ADEV")
    grid = np.logspace(np.log10(taus[0]), np.log10(taus[-1]), 200)
    ax.loglog(grid, identified.adev(grid), "-", color="#1e8449", lw=1.6,
              label="fitted model")
    ax.loglog(grid, truth.adev(grid), "--", color="#111111", lw=1.0,
              label="ground truth")
    ax.loglog(grid, identified.arw / 60.0 / np.sqrt(grid), ":", color="#c0392b", lw=0.9,
              label="ARW asymptote (N/sqrt(tau))")
    ax.loglog(grid, np.full_like(grid, 0.664 * identified.bias_instability / 3600.0),
              ":", color="#ef8a3c", lw=0.9, label="bias-instability floor")
    ax.set_xlabel("cluster time tau [s]")
    ax.set_ylabel("Allan deviation [deg/s]")
    ax.set_title("Static gyro log: 1 h at 100 Hz")
    ax.legend(frameon=False, fontsize=7, loc="lower left")
    figure.tight_layout()
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)


def plot_filters(sweep, tuned, naive, output_path) -> None:
    if not HAVE_MATPLOTLIB:
        raise RuntimeError("matplotlib is required for figures")
    _style()
    time_s = sweep["time_s"]
    figure, axes = plt.subplots(2, 1, figsize=(6.8, 5.0), sharex=True)
    axes[0].plot(time_s, tuned["error_deg"], color="#1e8449", lw=0.9,
                 label="tuned from Allan variance")
    axes[0].plot(time_s, naive["error_deg"], color="#c0392b", lw=0.9,
                 label="hand-tuned (4x optimistic)")
    axes[0].set_ylabel("tilt error [deg]")
    axes[0].set_title("Tilt filter on a 300 s sweep")
    axes[0].legend(frameon=False, fontsize=8)
    axes[1].semilogy(time_s, np.maximum(tuned["nees"], 1e-2), color="#1e8449", lw=0.8,
                     label="tuned NEES")
    axes[1].semilogy(time_s, np.maximum(naive["nees"], 1e-2), color="#c0392b", lw=0.8,
                     label="hand-tuned NEES")
    axes[1].axhline(2.0, color="#333333", lw=1.0, label="E[NEES] = 2")
    axes[1].set_ylabel("NEES")
    axes[1].set_xlabel("time [s]")
    axes[1].legend(frameon=False, fontsize=8)
    figure.tight_layout()
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)
