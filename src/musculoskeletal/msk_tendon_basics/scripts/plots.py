# -*- coding: utf-8 -*-
"""用 matplotlib 出图。图中文字用英文，避免服务器缺中文字体。"""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

__all__ = ["plot_isometric", "plot_tradeoff", "plot_release"]

_BLUE = "#2b6cb0"
_ORANGE = "#dd6b20"
_GREEN = "#2f855a"
_RED = "#c53030"
_GRAY = "#718096"


def _style(ax):
    ax.grid(True, alpha=0.25, linewidth=0.8)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def plot_isometric(result, path):
    acts = result["acts"]
    comp = result["compliant"]
    rigid = result["rigid"]

    fig, axes = plt.subplots(1, 3, figsize=(12, 3.6))

    ax = axes[0]
    ax.plot(acts, rigid["force"], "--", color=_GRAY, lw=1.8,
            label="rigid tendon")
    ax.plot(acts, comp["force"], "-o", color=_BLUE, ms=3, lw=1.6,
            label="elastic tendon")
    ax.set_xlabel("activation a")
    ax.set_ylabel("muscle force (N)")
    ax.set_title("Force transmission")
    ax.legend(frameon=False, fontsize=8)
    _style(ax)

    ax = axes[1]
    ax.plot(acts, rigid["fiber_length_norm"], "--", color=_GRAY, lw=1.8,
            label="rigid")
    ax.plot(acts, comp["fiber_length_norm"], "-o", color=_BLUE, ms=3, lw=1.6,
            label="elastic")
    ax.axhline(1.0, color=_ORANGE, lw=1.0, ls=":", label="l_opt")
    ax.set_xlabel("activation a")
    ax.set_ylabel("fiber length / l_opt")
    ax.set_title("Fiber operating length")
    ax.legend(frameon=False, fontsize=8)
    _style(ax)

    ax = axes[2]
    ax.plot(acts, [s * 100.0 for s in comp["strain"]], "-o", color=_GREEN,
            ms=3, lw=1.6, label="tendon strain")
    ax.set_xlabel("activation a")
    ax.set_ylabel("tendon strain (%)", color=_GREEN)
    ax.tick_params(axis="y", colors=_GREEN)
    ax.set_title("Tendon stretch and energy")
    ax2 = ax.twinx()
    ax2.plot(acts, comp["energy"], "-s", color=_RED, ms=3, lw=1.6,
             label="stored energy")
    ax2.set_ylabel("elastic energy (J)", color=_RED)
    ax2.tick_params(axis="y", colors=_RED)
    ax2.spines["top"].set_visible(False)
    _style(ax)

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_tradeoff(result, path):
    factors = result["factors"]
    force = result["force"]
    energy = result["energy"]

    fig, axes = plt.subplots(1, 2, figsize=(9, 3.8))

    ax = axes[0]
    ax.plot(factors, force, "-o", color=_BLUE, lw=1.6, ms=4)
    ax.set_xlabel("tendon stiffness (x nominal 30 kN/m)")
    ax.set_ylabel("force at a=1 (N)")
    ax.set_title("Stiffer tendon transmits more force")
    _style(ax)

    ax = axes[1]
    ax.plot(factors, energy, "-o", color=_RED, lw=1.6, ms=4)
    ax.set_xlabel("tendon stiffness (x nominal 30 kN/m)")
    ax.set_ylabel("stored energy at a=1 (J)")
    ax.set_title("Softer tendon stores more energy")
    _style(ax)

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_release(pair, path, hold_time=0.4):
    comp = pair["compliant"]["history"]
    rigid = pair["rigid"]["history"]

    fig, axes = plt.subplots(1, 3, figsize=(12, 3.6))

    ax = axes[0]
    ax.plot(rigid["t"], rigid["theta"], "--", color=_GRAY, lw=1.6,
            label="rigid")
    ax.plot(comp["t"], comp["theta"], "-", color=_BLUE, lw=1.8,
            label="elastic")
    ax.axvline(hold_time, color=_ORANGE, lw=1.0, ls=":")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("joint angle (rad)")
    ax.set_title("Release response")
    ax.legend(frameon=False, fontsize=8)
    _style(ax)

    ax = axes[1]
    ax.plot(rigid["t"], rigid["f_flex"], "--", color=_GRAY, lw=1.6,
            label="rigid")
    ax.plot(comp["t"], comp["f_flex"], "-", color=_BLUE, lw=1.8,
            label="elastic")
    ax.axvline(hold_time, color=_ORANGE, lw=1.0, ls=":")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("flexor tendon force (N)")
    ax.set_title("Force unloading")
    ax.legend(frameon=False, fontsize=8)
    _style(ax)

    ax = axes[2]
    ax.plot(comp["t"], comp["energy"], "-", color=_GREEN, lw=1.8,
            label="elastic tendon")
    ax.axvline(hold_time, color=_ORANGE, lw=1.0, ls=":")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("stored energy (J)")
    ax.set_title("Elastic energy")
    ax.legend(frameon=False, fontsize=8)
    _style(ax)

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
