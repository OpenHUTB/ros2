#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""可选绘图（matplotlib 缺失时自动跳过，只写 CSV，不影响退出码）。

坐标轴与图例用英文：matplotlib 默认字体没有中文字形，中文会渲染成方框。
"""
from __future__ import annotations

import os

import numpy as np

FIG_DPI = 120


def matplotlib_available():
    try:
        import matplotlib  # noqa: F401
        return True
    except Exception:  # noqa: BLE001
        return False


def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def plot_curves(tracks, ttc_threshold, save_path):
    """四联图：自车速度 / 净间距 / TTC / 制动量。"""
    plt = _plt()
    fig, axes = plt.subplots(4, 1, figsize=(9.0, 10.0), dpi=FIG_DPI, sharex=True)
    colors = {"baseline": "#1f77b4", "no_aeb": "#d62728", "high_speed": "#2ca02c"}

    for name, track in tracks.items():
        a = track.arrays()
        c = colors.get(name, "#555555")
        axes[0].plot(a["t"], a["v"], color=c, linewidth=1.6, label=name)
        axes[1].plot(a["t"], a["gap"], color=c, linewidth=1.6, label=name)
        axes[2].plot(a["t"], np.clip(a["ttc"], 0, ttc_threshold * 2.5), color=c,
                     linewidth=1.6, label=name)
        axes[3].plot(a["t"], a["brake"], color=c, linewidth=1.6, label=name)

    axes[0].set_ylabel("ego speed (m/s)")
    axes[0].set_title("AEB closed-loop: TTC-based proportional braking")
    axes[1].set_ylabel("gap to lead car (m)")
    axes[1].axhline(0.0, color="#999999", linestyle="--", linewidth=0.9)
    axes[2].set_ylabel("TTC (s)")
    axes[2].axhline(ttc_threshold, color="#ff7f0e", linestyle="--", linewidth=1.1,
                    label="TTC threshold")
    axes[3].set_ylabel("brake command (0-1)")
    axes[3].set_xlabel("time (s)")

    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(save_path, dpi=FIG_DPI)
    plt.close(fig)
    return save_path


def plot_sweep(rows, save_path):
    """min gap vs 初速度，按增益分组；0 线以下即碰撞。"""
    plt = _plt()
    fig, ax = plt.subplots(figsize=(8.0, 4.8), dpi=FIG_DPI)
    gains = sorted({r["gain"] for r in rows})
    for g in gains:
        pts = sorted([r for r in rows if r["gain"] == g], key=lambda r: r["ego_v"])
        ax.plot([p["ego_v"] for p in pts], [p["min_gap"] for p in pts],
                marker="o", linewidth=1.5, label="gain = %g%s" % (g, " (AEB off)" if g == 0 else ""))
    ax.axhline(0.0, color="#d62728", linestyle="--", linewidth=1.2,
               label="collision boundary (gap = 0)")
    ax.set_xlabel("initial ego speed (m/s)")
    ax.set_ylabel("minimum gap to lead car (m)")
    ax.set_title("AEB coverage: minimum gap under different gains and closing speeds")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(save_path, dpi=FIG_DPI)
    plt.close(fig)
    return save_path


def export_all(tracks, ttc_threshold, sweep_rows, save_dir):
    if not matplotlib_available():
        print("[提示] 未安装 matplotlib，跳过 PNG 绘图（CSV 已导出，不影响结果）。")
        print("       需要出图时：pip install matplotlib")
        return []
    os.makedirs(save_dir, exist_ok=True)
    out = [plot_curves(tracks, ttc_threshold,
                       os.path.join(save_dir, "aeb_curves.png"))]
    if sweep_rows:
        out.append(plot_sweep(sweep_rows, os.path.join(save_dir, "aeb_sweep.png")))
    return out
