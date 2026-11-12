"""Figures for the thruster allocation report."""

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


def plot_errors(data, allocators, output_path) -> None:
    if not HAVE_MATPLOTLIB:
        raise RuntimeError("matplotlib is required for figures")
    _style()
    figure, axes = plt.subplots(1, 2, figsize=(9.0, 3.8), sharey=True)
    labels = {
        "pseudo_inverse": ("pseudo-inverse", "#c0392b"),
        "clipped_pseudo_inverse": ("clipped pseudo-inverse", "#ef8a3c"),
        "qp_allocation": ("QP allocation", "#1e8449"),
    }
    for ax, scope, title in (
        (axes[0], "inside_wrenches", "inside the envelope"),
        (axes[1], "outside_wrenches", "outside the envelope (1.8x)"),
    ):
        from .core import error_norm
        for name, allocator in allocators.items():
            errors = np.sort(
                [error_norm(data["matrix"], allocator(data["matrix"], demand, data["max_thrust"]), demand)
                 for demand in data[scope]]
            )
            label, color = labels[name]
            ax.plot(errors, np.linspace(0.0, 1.0, errors.size), lw=1.5, color=color, label=label)
        ax.set_xlabel("wrench error norm")
        ax.set_title(title)
        ax.legend(frameon=False, fontsize=7)
    axes[0].set_ylabel("empirical CDF")
    figure.tight_layout()
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)


def plot_effort(summary, output_path) -> None:
    if not HAVE_MATPLOTLIB:
        raise RuntimeError("matplotlib is required for figures")
    _style()
    names = ["pseudo_inverse", "clipped_pseudo_inverse", "qp_allocation"]
    inside = [summary["inside"][name]["effort_mean"] for name in names]
    outside = [summary["outside"][name]["effort_mean"] for name in names]
    positions = np.arange(len(names))
    figure, ax = plt.subplots(figsize=(6.4, 3.8))
    ax.bar(positions - 0.18, inside, 0.36, color="#1e8449", label="inside envelope")
    ax.bar(positions + 0.18, outside, 0.36, color="#c0392b", label="outside envelope")
    ax.set_xticks(positions)
    ax.set_xticklabels(["pseudo-inverse", "clipped", "QP"], fontsize=8)
    ax.set_ylabel("mean effort sum(f^2) [N^2]")
    ax.set_title("Control effort")
    ax.legend(frameon=False)
    figure.tight_layout()
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)
