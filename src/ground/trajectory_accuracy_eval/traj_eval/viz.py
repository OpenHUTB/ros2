#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""可视化：真值/估计轨迹对比、位置误差随时间曲线。

matplotlib 为可选依赖：未安装时给出安装提示；`main.py --no-figures` 可跳过绘图。
"""

from __future__ import annotations

import os
from typing import Sequence

import numpy as np

from .se2 import Pose, align_umeyama, positions, transform_poses


def _plt():
    try:
        import matplotlib
        matplotlib.use("Agg")          # 无显示器环境也能出图
        import matplotlib.pyplot as plt
        return plt
    except ImportError as exc:  # pragma: no cover - 环境相关
        raise RuntimeError("缺少 matplotlib，无法出图。请 pip3 install matplotlib，"
                           "或加 --no-figures 跳过绘图。") from exc


def _setup_cjk(plt) -> bool:
    """尝试启用中文字体，避免标题出现方块。"""
    from matplotlib import font_manager
    wanted = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "WenQuanYi Zen Hei"]
    have = {f.name for f in font_manager.fontManager.ttflist}
    for name in wanted:
        if name in have:
            plt.rcParams["font.sans-serif"] = [name]
            plt.rcParams["axes.unicode_minus"] = False
            return True
    return False


def plot_trajectories(gt: Sequence[Pose], est: Sequence[Pose], output: str,
                      aligned: bool = True) -> str:
    """画真值 / 原始估计 / 对齐后估计三条轨迹。"""
    plt = _plt()
    cjk = _setup_cjk(plt)
    g = positions(gt)
    e = positions(est)

    fig, ax = plt.subplots(figsize=(8.2, 6.4))
    ax.plot(g[:, 0], g[:, 1], "k-", linewidth=2.2,
            label="真值 ground truth" if cjk else "ground truth")
    ax.plot(e[:, 0], e[:, 1], color="#d62728", linewidth=1.4, alpha=0.9,
            label="里程计估计" if cjk else "odometry")
    if aligned:
        al = align_umeyama(e, g, with_scale=False)
        ea = al.apply_points(e)
        ax.plot(ea[:, 0], ea[:, 1], "--", color="#1f77b4", linewidth=1.4, alpha=0.9,
                label="刚体对齐后" if cjk else "after rigid alignment")
    ax.set_xlabel("x / m")
    ax.set_ylabel("y / m")
    ax.set_title("轨迹对比" if cjk else "trajectory comparison")
    ax.axis("equal")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(output)) or ".", exist_ok=True)
    fig.savefig(output, dpi=130)
    plt.close(fig)
    return output


def plot_error_curve(gt: Sequence[Pose], est: Sequence[Pose], output: str,
                     times: Sequence[float] | None = None) -> str:
    """画对齐后的位置误差与朝向误差随时间的曲线。"""
    plt = _plt()
    cjk = _setup_cjk(plt)
    g = positions(gt)
    al = align_umeyama(positions(est), g, with_scale=False)
    ea = al.apply_points(positions(est))
    err = np.hypot(*(ea - g).T)
    est_yaw = al.apply_yaws(np.array([p.yaw for p in est]))
    gt_yaw = np.array([p.yaw for p in gt])
    dyaw = np.degrees(np.abs((est_yaw - gt_yaw + np.pi) % (2 * np.pi) - np.pi))
    if times is None:
        times = np.arange(len(err))

    fig, axes = plt.subplots(2, 1, figsize=(9.5, 6.2), sharex=True)
    axes[0].plot(times, err, color="#1f77b4", linewidth=1.5)
    axes[0].set_ylabel("位置误差 / m" if cjk else "position error / m")
    axes[0].set_title("对齐后的误差曲线" if cjk else "error after alignment")
    axes[0].set_ylim(0.0, max(float(err.max()) * 1.15, 1e-3))
    axes[0].grid(alpha=0.3)

    axes[1].plot(times, dyaw, color="#d62728", linewidth=1.5)
    axes[1].set_ylabel("朝向误差 / deg" if cjk else "yaw error / deg")
    axes[1].set_xlabel("时间 t / s" if cjk else "time t / s")
    axes[1].grid(alpha=0.3)
    # 航向误差通常只有 1e-3 度量级（尺度误差只改变距离、不改变朝向）。
    # 不固定纵轴范围的话，matplotlib 会把 1e-14 的浮点噪声放大成"尖峰"，
    # 图上看起来像有几十度的误差，与真实数据完全不符——这里显式设定可读范围。
    yaw_max = float(dyaw.max())
    if yaw_max < 0.05:
        axes[1].set_ylim(-0.02, 0.05)
        axes[1].ticklabel_format(axis="y", style="plain", useOffset=False)
        axes[1].text(0.015, 0.88,
                     (f"航向误差恒为 {yaw_max:.4f}°\n尺度误差只影响距离，不影响航向"
                      if cjk else
                      f"yaw error is constant {yaw_max:.4f} deg\n"
                      "scale error affects distance only"),
                     transform=axes[1].transAxes, fontsize=10, va="top",
                     bbox=dict(boxstyle="round,pad=0.35", fc="#fff3f3", ec="#d62728", lw=0.8))
    else:
        axes[1].set_ylim(0.0, yaw_max * 1.15)

    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(output)) or ".", exist_ok=True)
    fig.savefig(output, dpi=130)
    plt.close(fig)
    return output


def plot_sweep(scales: Sequence[float], drift: Sequence[float], ate_rmse: Sequence[float],
               output: str) -> str:
    """画"轮半径尺度误差 → 漂移率/ATE"的敏感性曲线。"""
    plt = _plt()
    cjk = _setup_cjk(plt)
    fig, ax1 = plt.subplots(figsize=(8.6, 5.0))
    ax1.plot(scales, drift, "o-", color="#1f77b4", label="终点漂移率 / %" if cjk else "drift / %")
    ax1.set_xlabel("轮半径尺度系数" if cjk else "wheel scale factor")
    ax1.set_ylabel("终点漂移率 / %" if cjk else "drift / %", color="#1f77b4")
    ax1.tick_params(axis="y", labelcolor="#1f77b4")
    ax1.grid(alpha=0.3)

    ax2 = ax1.twinx()
    ax2.plot(scales, ate_rmse, "s--", color="#d62728", label="ATE RMSE / m" if cjk else "ATE RMSE / m")
    ax2.set_ylabel("ATE RMSE / m", color="#d62728")
    ax2.tick_params(axis="y", labelcolor="#d62728")

    ax1.set_title("尺度误差对里程计精度的影响" if cjk else "effect of scale error")
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(output)) or ".", exist_ok=True)
    fig.savefig(output, dpi=130)
    plt.close(fig)
    return output
