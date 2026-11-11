"""绘图：正弦跟踪、阶跃响应、阶跃 GIF。

所有图内文字（标题、轴标签、图例、注释）都用英文，避免 Windows 上
中文字体缺失导致方块；代码注释与 README 用中文。
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # 必须在 pyplot 之前设置，保证无界面环境也能出图
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle

from .simulation import SimulationResult

FIG_DPI = 150
COLOR_REF = "#222222"
COLOR_ACTUAL = "#1f77b4"
COLOR_ERROR = "#d62728"
COLOR_BAND = "#9ecae1"
COLOR_BAND_WIDE = "#e6f2fb"


def _style(ax: plt.Axes) -> None:
    ax.grid(True, alpha=0.3, linewidth=0.6)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.tick_params(labelsize=9)


def plot_sine(
    result: SimulationResult,
    metrics: dict[str, float],
    path: str | Path,
    title: str = "Sine tracking (bias 30 deg, amplitude 40 deg, 0.4 Hz)",
) -> Path:
    """正弦段：参考 vs 实际 + 误差，两联图。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    t = result.t
    fig, (ax1, ax2) = plt.subplots(
        2,
        1,
        figsize=(10, 6.4),
        sharex=True,
        gridspec_kw={"height_ratios": [3.0, 1.15]},
    )
    ax1.plot(t, result.reference_deg, "--", color=COLOR_REF, lw=1.4, label="Reference")
    ax1.plot(t, result.theta_deg, "-", color=COLOR_ACTUAL, lw=1.8, label="Actual")
    ax1.set_ylabel("Knee angle [deg]", fontsize=11)
    ax1.set_title(title, fontsize=12)
    ax1.legend(loc="upper right", fontsize=9, frameon=True)
    text = (
        f"RMSE = {metrics['rmse_deg']:.2f} deg\n"
        f"Max |error| = {metrics['max_abs_error_deg']:.2f} deg"
    )
    ax1.text(
        0.015,
        0.04,
        text,
        transform=ax1.transAxes,
        fontsize=9,
        va="bottom",
        bbox={"boxstyle": "round,pad=0.35", "fc": "white", "ec": "#cccccc", "alpha": 0.9},
    )
    _style(ax1)

    ax2.plot(t, result.error_deg, "-", color=COLOR_ERROR, lw=1.2)
    ax2.axhline(0.0, color="black", lw=0.8, alpha=0.6)
    ax2.set_xlabel("Time [s]", fontsize=11)
    ax2.set_ylabel("Tracking error [deg]", fontsize=11)
    _style(ax2)

    fig.tight_layout()
    fig.savefig(path, dpi=FIG_DPI)
    plt.close(fig)
    return path


def plot_step(
    result: SimulationResult,
    metrics: dict[str, float | bool | None],
    target_deg: float,
    step_time: float,
    path: str | Path,
    band_wide_deg: float = 5.0,
) -> Path:
    """阶跃响应：目标、容差带、实际角与误差；时间以阶跃时刻为 0。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    t = result.t - step_time
    mask = t >= -0.25
    t = t[mask]
    theta = result.theta_deg[mask]
    ref = result.reference_deg[mask]
    error = result.error_deg[mask]

    fig, (ax1, ax2) = plt.subplots(
        2,
        1,
        figsize=(10, 6.4),
        sharex=True,
        gridspec_kw={"height_ratios": [3.0, 1.15]},
    )
    band = float(metrics["settling_band_deg"])
    ax1.axhspan(target_deg - band_wide_deg, target_deg + band_wide_deg,
                color=COLOR_BAND_WIDE, label=f"+/-{band_wide_deg:.0f} deg band")
    ax1.axhspan(target_deg - band, target_deg + band, color=COLOR_BAND,
                alpha=0.8, label=f"+/-2% band ({band:.2f} deg)")
    ax1.axhline(target_deg, color=COLOR_REF, ls="--", lw=1.2, label="Target")
    ax1.plot(t, theta, "-", color=COLOR_ACTUAL, lw=2.0, label="Actual")
    ax1.axvline(0.0, color="#888888", lw=0.8, ls=":")
    ax1.set_ylabel("Knee angle [deg]", fontsize=11)
    ax1.set_title("Step response: 20 deg -> 50 deg", fontsize=12)
    ax1.legend(loc="lower right", fontsize=8, frameon=True)
    _style(ax1)

    settle = metrics["settling_time_s"]
    settle_text = f"{float(settle):.2f} s" if settle is not None else "not entered"
    wide = metrics["wide_band_entry_s"]
    wide_text = f"{float(wide):.2f} s" if wide is not None else "not entered"
    text = (
        f"Overshoot = {float(metrics['overshoot_percent']):.2f} %\n"
        f"Settling (+/-2%): {settle_text}\n"
        f"First entry +/-{band_wide_deg:.0f} deg: {wide_text}\n"
        f"Steady-state error = {float(metrics['steady_state_error_deg']):+.2f} deg"
    )
    ax1.text(
        0.015,
        0.96,
        text,
        transform=ax1.transAxes,
        fontsize=8.5,
        va="top",
        bbox={"boxstyle": "round,pad=0.35", "fc": "white", "ec": "#cccccc", "alpha": 0.9},
    )

    ax2.plot(t, error, "-", color=COLOR_ERROR, lw=1.2)
    ax2.axhline(0.0, color="black", lw=0.8, alpha=0.6)
    ax2.axhspan(-band, band, color=COLOR_BAND, alpha=0.45)
    ax2.set_xlabel("Time from step [s]", fontsize=11)
    ax2.set_ylabel("Tracking error [deg]", fontsize=11)
    ax2.set_xlim(max(-0.25, t[0]), max(t[-1], 0.1))
    _style(ax2)

    fig.tight_layout()
    fig.savefig(path, dpi=FIG_DPI)
    plt.close(fig)
    return path


def animate_step(
    result: SimulationResult,
    step_time: float,
    path: str | Path,
    fps: int = 15,
    dpi: int = 80,
    max_frames: int = 120,
) -> Path:
    """阶跃响应的 GIF：左侧膝部示意，右侧角度-时间曲线。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    t_rel = result.t - step_time
    mask = t_rel >= -0.25
    t = t_rel[mask]
    theta = result.theta_deg[mask]
    target = result.reference_deg[mask]
    n = t.size
    if n == 0:
        raise ValueError("没有可绘制的数据")
    frame_idx = np.unique(
        np.linspace(0, n - 1, num=min(n, max(2, max_frames))).astype(int)
    )

    fig, (ax_knee, ax_curve) = plt.subplots(
        1, 2, figsize=(8.4, 4.0), gridspec_kw={"width_ratios": [1.0, 1.45]}
    )
    y_lo = float(min(theta.min(), target.min())) - 6.0
    y_hi = float(max(theta.max(), target.max())) + 6.0
    x_lo = float(t.min())
    x_hi = float(t.max())

    def draw_frame(frame_number: int) -> list:
        idx = int(frame_idx[frame_number])
        angle = float(theta[idx])
        ref_angle = float(target[idx])

        ax_knee.clear()
        ax_knee.set_xlim(-1.15, 1.15)
        ax_knee.set_ylim(-1.2, 0.45)
        ax_knee.set_aspect("equal")
        ax_knee.axis("off")
        ax_knee.set_title("Knee schematic (sagittal)", fontsize=10)

        # 大腿固定段与膝关节
        ax_knee.plot([-1.0, 0.0], [0.12, 0.0], color="#555555", lw=6, solid_capstyle="round")
        ax_knee.add_patch(Circle((0.0, 0.0), 0.035, color="#333333", zorder=5))
        # 参考小腿（虚线）与当前小腿（实线）
        rad_ref = math.radians(ref_angle)
        ax_knee.plot(
            [0.0, math.sin(rad_ref)], [0.0, -math.cos(rad_ref)],
            ls="--", color="#999999", lw=1.2, zorder=2,
        )
        rad = math.radians(angle)
        ax_knee.plot(
            [0.0, math.sin(rad)], [0.0, -math.cos(rad)],
            color=COLOR_ACTUAL, lw=4, solid_capstyle="round", zorder=3,
        )
        ax_knee.add_patch(
            Circle((math.sin(rad), -math.cos(rad)), 0.055, color="#1f77b4", zorder=4)
        )
        ax_knee.text(
            0.0, 0.30, f"Knee angle: {angle:5.1f} deg", ha="center", fontsize=10
        )
        ax_knee.text(
            0.0, 0.20, f"Target: {ref_angle:5.1f} deg", ha="center", fontsize=9, color="#666666"
        )

        ax_curve.clear()
        ax_curve.set_xlim(x_lo, x_hi)
        ax_curve.set_ylim(y_lo, y_hi)
        ax_curve.plot(t, target, "--", color=COLOR_REF, lw=1.1, label="Reference")
        ax_curve.plot(t[: idx + 1], theta[: idx + 1], "-", color=COLOR_ACTUAL, lw=1.8, label="Actual")
        ax_curve.plot([t[idx]], [angle], "o", color=COLOR_ACTUAL, ms=4)
        ax_curve.axvline(0.0, color="#888888", lw=0.7, ls=":")
        ax_curve.set_xlabel("Time from step [s]", fontsize=9)
        ax_curve.set_ylabel("Knee angle [deg]", fontsize=9)
        ax_curve.set_title("Step response: 20 deg -> 50 deg", fontsize=10)
        ax_curve.legend(loc="lower right", fontsize=7, frameon=True)
        _style(ax_curve)
        fig.tight_layout()
        return []

    anim = FuncAnimation(
        fig,
        draw_frame,
        frames=len(frame_idx),
        interval=1000.0 / max(fps, 1),
        blit=False,
        repeat=False,
    )
    anim.save(path, writer=PillowWriter(fps=fps), dpi=dpi)
    plt.close(fig)
    return path
