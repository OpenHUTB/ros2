"""Figures for the wind-estimation report."""

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


def plot_maneuvers(config, output_path) -> None:
    if not HAVE_MATPLOTLIB:
        raise RuntimeError("matplotlib is required for figures")
    _style()
    from .core import estimate_wind
    from .scene import MANEUVERS, simulate_maneuver

    figure, ax = plt.subplots(figsize=(6.2, 5.0))
    colors = {"hover": "#c0392b", "straight": "#ef8a3c", "reciprocal": "#2c6fbb", "circle": "#1e8449"}
    ax.plot([0.0, config.wind[0]], [0.0, config.wind[1]], color="#111111", lw=2.0,
            label="true wind")
    for maneuver in MANEUVERS:
        data = simulate_maneuver(config, maneuver)
        try:
            estimate = estimate_wind(data["ground_velocity"], data["headings"],
                                     sigma_m_s=config.gps_noise_m_s)
            vector = estimate.wind
        except ValueError:
            vector = np.mean(data["ground_velocity"], axis=0)
        ax.annotate("", xy=vector, xytext=(0.0, 0.0),
                    arrowprops={"arrowstyle": "->", "color": colors[maneuver], "lw": 1.6})
        ax.scatter([vector[0]], [vector[1]], color=colors[maneuver], s=18,
                   label=f"{maneuver} estimate")
    ax.set_xlabel("east wind [m/s]")
    ax.set_ylabel("north wind [m/s]")
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Wind estimate per manoeuvre")
    ax.legend(frameon=False, fontsize=7)
    figure.tight_layout()
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)


def plot_noise(sweep, output_path) -> None:
    if not HAVE_MATPLOTLIB:
        raise RuntimeError("matplotlib is required for figures")
    _style()
    sigmas = np.array([block["gps_noise_m_s"] for block in sweep.values()])
    errors = np.array([block["speed_rmse_m_s"] for block in sweep.values()])
    figure, ax = plt.subplots(figsize=(6.2, 3.8))
    ax.plot(sigmas, errors, "o-", color="#1e8449")
    ax.set_xlabel("GPS velocity noise sigma [m/s]")
    ax.set_ylabel("wind speed error [m/s]")
    ax.set_title("Sensitivity to GPS noise (circle manoeuvre)")
    figure.tight_layout()
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)
