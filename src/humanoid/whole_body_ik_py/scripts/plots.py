"""全部 PNG 图：奇异点步长、自适应阻尼、可操作性曲线、速度椭球、IK 收敛。"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "axes.grid": True,
        "grid.alpha": 0.25,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "figure.facecolor": "white",
    }
)

BLUE = "#2b6cb0"
ORANGE = "#dd6b20"
GREEN = "#2f855a"
RED = "#c53030"
GRAY = "#718096"


def plot_singularity(rows: list[dict], path: str | Path) -> None:
    elbow = np.array([r["elbow"] for r in rows])
    pinv = np.array([r["step_pinv"] for r in rows])
    dls = np.array([r["step_dls"] for r in rows])

    fig, axes = plt.subplots(1, 2, figsize=(11.0, 4.4), constrained_layout=True)
    ax0, ax1 = axes
    ax0.plot(elbow, pinv, "o-", color=RED, lw=1.8, label="pseudoinverse")
    ax0.plot(elbow, dls, "s-", color=BLUE, lw=1.8, label="DLS (adaptive $\\lambda$)")
    ax0.set_yscale("log")
    ax0.set_xlabel("Elbow angle $q_1$ [rad]")
    ax0.set_ylabel("|$\\Delta q$| for 0.01 m radial error")
    ax0.set_title("Joint step near full extension: pinv blows up, DLS stays bounded")
    ax0.legend(loc="best")
    ax0.invert_xaxis()

    lam = np.array([r["adaptive_lambda"] for r in rows])
    smin = np.array([r["sigma_min"] for r in rows])
    ax1.plot(smin, lam, "o-", color=GREEN, lw=1.8)
    ax1.set_xscale("log")
    ax1.set_xlabel("$\\sigma_{min}$")
    ax1.set_ylabel("adaptive $\\lambda$")
    ax1.set_title("Adaptive damping: $\\lambda$ rises as $\\sigma_{min}$ falls")
    ax1.invert_xaxis()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_manipulability(sweep: dict, l1: float, l2: float, path: str | Path) -> None:
    elbow = np.asarray(sweep["elbow"])
    numeric = np.asarray(sweep["numeric"])
    closed = np.asarray(sweep["closed_form"])
    cond = np.asarray(sweep["condition"])

    fig, axes = plt.subplots(2, 1, figsize=(9.6, 6.6), sharex=True,
                             constrained_layout=True)
    ax0, ax1 = axes
    ax0.plot(elbow, closed, color=GRAY, lw=5.0, alpha=0.35,
             label="closed form $|l_1 l_2 \\sin q_1|$")
    ax0.plot(elbow, numeric, color=BLUE, lw=1.8, label="numeric $\\sqrt{\\det(JJ^T)}$")
    ax0.axvline(np.pi / 2.0, color=GREEN, ls=":", lw=1.2)
    ax0.set_ylabel("Manipulability $w$")
    ax0.set_title(f"2R arm manipulability  ($l_1$ = {l1:.2f} m, $l_2$ = {l2:.2f} m)")
    ax0.legend(loc="lower center")

    ax1.plot(elbow, np.minimum(cond, 1e4), color=ORANGE, lw=1.8)
    ax1.set_yscale("log")
    ax1.set_xlabel("Elbow angle $q_1$ [rad]")
    ax1.set_ylabel("Condition number $\\sigma_{max}/\\sigma_{min}$")
    ax1.set_title("Condition number diverges at full extension")
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_ellipsoids(cases: list[dict], path: str | Path) -> None:
    n = len(cases)
    fig, axes = plt.subplots(1, n, figsize=(4.0 * n, 4.0), constrained_layout=True)
    if n == 1:
        axes = [axes]
    for ax, case in zip(axes, cases):
        ell = case["ellipsoid"]
        pts = ell["points"]
        ax.fill(pts[0], pts[1], color=BLUE, alpha=0.18)
        ax.plot(pts[0], pts[1], color=BLUE, lw=1.6)
        axes_len = ell["semi_axes"]
        ax.annotate("", xy=(axes_len[0] * ell["major_axis"][0],
                            axes_len[0] * ell["major_axis"][1]),
                    xytext=(0, 0),
                    arrowprops=dict(arrowstyle="->", color=RED, lw=1.6))
        ax.annotate("", xy=(axes_len[1] * ell["minor_axis"][0],
                            axes_len[1] * ell["minor_axis"][1]),
                    xytext=(0, 0),
                    arrowprops=dict(arrowstyle="->", color=GREEN, lw=1.6))
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("$v_x$ [m/s]")
        ax.set_ylabel("$v_y$ [m/s]")
        ax.set_title(
            f"$q_1$ = {case['elbow']:.2f} rad\n"
            f"$w$ = {case['manipulability']:.4f}, "
            f"axes = {axes_len[0]:.3f}/{axes_len[1]:.3f}",
            fontsize=9,
        )
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_ik_tracking(errors: np.ndarray, iterations: np.ndarray, plane: str,
                     path: str | Path) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(9.4, 6.2), sharex=True,
                             constrained_layout=True)
    ax0, ax1 = axes
    k = np.arange(errors.size) + 1
    ax0.semilogy(k, np.maximum(errors, 1e-16), "o-", color=BLUE, lw=1.6, ms=4)
    ax0.axhline(1e-6, color=RED, ls="--", lw=1.0, label="tolerance 1e-6")
    ax0.set_ylabel("End-effector error [m]")
    ax0.set_title(f"7-DOF arm circle tracking ({plane} plane): "
                  f"max = {errors.max():.2e} m")
    ax0.legend(loc="best")

    ax1.bar(k, iterations, color=ORANGE, alpha=0.85, width=0.7)
    ax1.set_xlabel("Waypoint index")
    ax1.set_ylabel("IK iterations")
    ax1.set_title(f"mean = {iterations.mean():.1f}, max = {iterations.max()}")
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_arm_snapshot(chain, q, path: str | Path, title: str) -> None:
    """一张静态侧视图，用于 README 快速展示。"""
    fig, ax = plt.subplots(figsize=(5.0, 4.8), constrained_layout=True)
    pts = chain.link_points(q)
    ax.plot(pts[:, 0], pts[:, 2], "-o", color=BLUE, lw=2.6, ms=5,
            mfc="#1a202c", mec="#1a202c")
    ax.scatter([pts[0, 0]], [pts[0, 2]], s=80, marker="s", color="#1a202c")
    ax.scatter([pts[-1, 0]], [pts[-1, 2]], s=60, color=RED)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-0.55, 0.55)
    ax.set_ylim(-0.20, 0.55)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Z [m]")
    ax.set_title(title, fontsize=10)
    fig.savefig(path, dpi=150)
    plt.close(fig)
