"""Figures for the time-offset report."""

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


def plot_signals(pair, output_path) -> None:
    if not HAVE_MATPLOTLIB:
        raise RuntimeError("matplotlib is required for figures")
    _style()
    figure, axes = plt.subplots(2, 1, figsize=(7.2, 4.6), sharex=True)
    time_s = pair["time_s"]
    axes[0].plot(time_s, pair["gyro"], color="#2c6fbb", lw=0.9, label="gyro")
    axes[0].plot(time_s, pair["visual"], color="#c0392b", lw=0.8, alpha=0.7,
                 label="visual rate (delayed)")
    axes[0].set_ylabel("angular rate [rad/s]")
    axes[0].legend(frameon=False, fontsize=8)
    axes[0].set_title("Gyro and visual-rate streams")

    from .core import cross_correlation

    lags = (np.arange(-80, 81) * pair["dt"]) * 1000.0
    values = cross_correlation(pair["gyro"], pair["visual"], 80)
    axes[1].plot(lags, values, color="#1e8449", lw=1.2)
    axes[1].axvline(pair["true_offset_s"] * 1000.0, color="#111111", ls="--", lw=0.9,
                    label="true offset")
    axes[1].set_xlabel("lag [ms]")
    axes[1].set_ylabel("normalised correlation")
    axes[1].legend(frameon=False, fontsize=8)
    figure.tight_layout()
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)


def plot_errors(rows, output_path) -> None:
    if not HAVE_MATPLOTLIB:
        raise RuntimeError("matplotlib is required for figures")
    _style()
    errors = np.array([row["error_ms"] for row in rows])
    figure, axes = plt.subplots(1, 2, figsize=(9.0, 3.6))
    axes[0].hist(errors, bins=30, color="#1e8449", alpha=0.8)
    axes[0].axvline(0.0, color="#111111", lw=0.9)
    axes[0].set_xlabel("offset error [ms]")
    axes[0].set_ylabel("trials")
    axes[0].set_title("Estimation error (200 trials)")
    axes[1].plot(np.sort(np.abs(errors)), np.linspace(0.0, 1.0, errors.size),
                 color="#1e8449", lw=1.4)
    axes[1].axvline(np.percentile(np.abs(errors), 95), color="#c0392b", ls="--", lw=0.9,
                    label="95th percentile")
    axes[1].set_xlabel("absolute offset error [ms]")
    axes[1].set_ylabel("empirical CDF")
    axes[1].legend(frameon=False, fontsize=8)
    figure.tight_layout()
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)
