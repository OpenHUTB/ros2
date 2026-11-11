"""Figures for the RTK ambiguity-resolution report."""

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


def plot_error_cdf(rows, output_path) -> None:
    if not HAVE_MATPLOTLIB:
        raise RuntimeError("matplotlib is required for figures")
    _style()
    float_errors = np.sort(1000.0 * np.array([row["float_error_m"] for row in rows]))
    fixed_errors = np.sort(1000.0 * np.array([row["fixed_error_m"] for row in rows]))
    fraction = np.linspace(0.0, 1.0, len(float_errors))
    figure, ax = plt.subplots(figsize=(6.2, 4.0))
    ax.plot(float_errors, fraction, color="#c0392b", lw=1.6, label="float solution")
    ax.plot(fixed_errors, fraction, color="#1e8449", lw=1.6, label="fixed solution")
    ax.set_xscale("log")
    ax.set_xlabel("baseline error [mm]")
    ax.set_ylabel("empirical CDF")
    ax.set_title("Baseline error per fix window (20 epochs)")
    ax.legend(frameon=False)
    figure.tight_layout()
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)


def plot_window_sweep(sweep, output_path) -> None:
    if not HAVE_MATPLOTLIB:
        raise RuntimeError("matplotlib is required for figures")
    _style()
    windows = np.array([int(key) for key in sweep])
    success = np.array([block["success_rate_pct"] for block in sweep.values()])
    wrong = np.array([block["wrong_fix_pct"] for block in sweep.values()])
    figure, ax = plt.subplots(figsize=(6.2, 4.0))
    ax.plot(windows, success, "o-", color="#1e8449", label="correct fix rate (all windows)")
    ax.plot(windows, wrong, "s--", color="#c0392b", label="wrong fix rate (all windows)")
    ax.set_xscale("log")
    ax.set_xticks(windows)
    ax.set_xticklabels([str(value) for value in windows])
    ax.minorticks_off()
    ax.set_xlabel("epochs per fix window")
    ax.set_ylabel("rate [%]")
    ax.set_ylim(-3.0, 105.0)
    ax.set_title("Ambiguity resolution versus window length")
    ax.legend(frameon=False)
    figure.tight_layout()
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)
