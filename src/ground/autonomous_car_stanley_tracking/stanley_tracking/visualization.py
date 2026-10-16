#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""可视化：轨迹对比图、横向偏差曲线、指标柱状图与跟踪过程动图。

matplotlib 是可选依赖：未安装时本模块的 `require_matplotlib()`
会给出明确的安装提示，而 `main.py` 的主流程（数值仿真与指标输出）仍可运行。
"""

from __future__ import annotations

import os
from typing import Dict, List, Optional, Sequence

import numpy as np

from .simulator import Metrics, SimulationTrace


def require_matplotlib():
    """导入 matplotlib，未安装时抛出带安装说明的错误。"""
    try:
        import matplotlib
        matplotlib.use("Agg")  # 无显示环境也能出图
        import matplotlib.pyplot as plt
        return matplotlib, plt
    except ImportError as exc:  # pragma: no cover - 环境相关
        raise RuntimeError(
            "缺少 matplotlib，无法生成效果图。请执行:\n"
            "    pip3 install matplotlib\n"
            "或使用 --no-figures 跳过绘图，仅输出数值指标。"
        ) from exc


def configure_chinese_font(plt) -> bool:
    """尝试启用中文字体，避免图例与标题出现方块。"""
    from matplotlib import font_manager

    candidates = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "WenQuanYi Zen Hei"]
    available = {f.name for f in font_manager.fontManager.ttflist}
    for name in candidates:
        if name in available:
            plt.rcParams["font.sans-serif"] = [name]
            plt.rcParams["axes.unicode_minus"] = False
            return True
    # 找不到中文字体时退回英文标签（调用方用 _label 控制）
    return False


class _Labels:
    """中英文字段名：系统无中文字体时自动退回英文。"""

    def __init__(self, chinese: bool) -> None:
        self.chinese = chinese

    def __call__(self, zh: str, en: str) -> str:
        return zh if self.chinese else en


def plot_trajectories(traces: Sequence[SimulationTrace], path, output: str,
                      title_zh: str = "横向控制轨迹对比", title_en: str = "Lateral control comparison") -> str:
    """把参考路径与各控制器的实际轨迹画在一张图上。"""
    _, plt = require_matplotlib()
    labels = _Labels(configure_chinese_font(plt))

    fig, ax = plt.subplots(figsize=(9.0, 7.0))
    sampled = path.sample(600)
    ax.plot(sampled[:, 0], sampled[:, 1], "k--", linewidth=2.0,
            label=labels("参考路径", "reference"))

    colors = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e"]
    for index, trace in enumerate(traces):
        ax.plot(trace.x, trace.y, color=colors[index % len(colors)], linewidth=1.8,
                label=f"{trace.controller} ({trace.scenario})")
        ax.plot(trace.x[0], trace.y[0], "o", color=colors[index % len(colors)], markersize=6)
        ax.plot(trace.x[-1], trace.y[-1], "s", color=colors[index % len(colors)], markersize=6)

    ax.set_xlabel(labels("x / m", "x / m"))
    ax.set_ylabel(labels("y / m", "y / m"))
    ax.set_title(labels(title_zh, title_en))
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    fig.savefig(output, dpi=130)
    plt.close(fig)
    return output


def plot_errors(traces: Sequence[SimulationTrace], output: str,
                title_zh: str = "横向偏差与航向偏差", title_en: str = "Cross-track and heading error") -> str:
    """横向偏差与航向偏差随时间的曲线（双联图）。"""
    _, plt = require_matplotlib()
    labels = _Labels(configure_chinese_font(plt))

    fig, axes = plt.subplots(2, 1, figsize=(10.0, 6.5), sharex=True)
    colors = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e"]
    for index, trace in enumerate(traces):
        color = colors[index % len(colors)]
        axes[0].plot(trace.time, trace.cross_track_error, color=color, linewidth=1.4,
                     label=f"{trace.controller}")
        axes[1].plot(trace.time, np.degrees(trace.heading_error), color=color, linewidth=1.4,
                     label=f"{trace.controller}")

    axes[0].axhline(0.0, color="k", linewidth=0.8)
    axes[0].axhline(0.2, color="gray", linestyle=":", linewidth=0.9)
    axes[0].axhline(-0.2, color="gray", linestyle=":", linewidth=0.9)
    axes[0].set_ylabel(labels("横向偏差 e / m", "cross-track error e / m"))
    axes[0].set_title(labels(title_zh, title_en))
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(fontsize=8, ncol=3)

    axes[1].set_xlabel(labels("时间 t / s", "time t / s"))
    axes[1].set_ylabel(labels("航向偏差 psi / deg", "heading error psi / deg"))
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    fig.savefig(output, dpi=130)
    plt.close(fig)
    return output


def plot_metrics(metrics: Sequence[Metrics], output: str,
                 title_zh: str = "跟踪性能指标对比", title_en: str = "Tracking metrics") -> str:
    """指标柱状图：平均绝对横向偏差、最大偏差、稳态偏差、转向抖动。"""
    _, plt = require_matplotlib()
    labels = _Labels(configure_chinese_font(plt))

    names = [f"{m.controller}\n{m.scenario}" for m in metrics]
    panels = [
        (labels("平均绝对横向偏差 / m", "mean |e| / m"), [m.cross_track_mean_abs for m in metrics], "#1f77b4"),
        (labels("最大绝对横向偏差 / m", "max |e| / m"), [m.cross_track_max_abs for m in metrics], "#d62728"),
        (labels("稳态横向偏差 / m", "steady |e| / m"), [m.cross_track_steady_abs for m in metrics], "#2ca02c"),
        (labels("转向抖动 RMS / (deg/s)", "steering rate RMS / (deg/s)"),
         [m.steering_rate_rms_deg for m in metrics], "#ff7f0e"),
    ]

    fig, axes = plt.subplots(2, 2, figsize=(11.0, 7.0))
    for ax, (label, values, color) in zip(axes.ravel(), panels):
        positions = np.arange(len(values))
        ax.bar(positions, values, color=color, alpha=0.85)
        ax.set_xticks(positions)
        ax.set_xticklabels(names, fontsize=7.5)
        ax.set_ylabel(label, fontsize=9)
        ax.grid(True, axis="y", alpha=0.3)
        for pos, value in zip(positions, values):
            ax.text(pos, value, f"{value:.3f}", ha="center", va="bottom", fontsize=7.5)
    fig.suptitle(labels(title_zh, title_en))
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    fig.savefig(output, dpi=130)
    plt.close(fig)
    return output


def animate_tracking(trace: SimulationTrace, path, output: str, fps: int = 25,
                     frame_stride: int = 5) -> str:
    """生成跟踪过程动图 GIF（轨迹逐渐生长 + 车体朝向箭头）。

    依赖 imageio；未安装时抛出带安装说明的错误。
    """
    try:
        import imageio.v2 as imageio
    except ImportError as exc:  # pragma: no cover - 环境相关
        raise RuntimeError("生成动图需要 imageio，请执行: pip3 install imageio") from exc

    _, plt = require_matplotlib()
    labels = _Labels(configure_chinese_font(plt))

    sampled = path.sample(600)
    frames: List[np.ndarray] = []
    x_min = min(trace.x.min(), sampled[:, 0].min()) - 5.0
    x_max = max(trace.x.max(), sampled[:, 0].max()) + 5.0
    y_min = min(trace.y.min(), sampled[:, 1].min()) - 5.0
    y_max = max(trace.y.max(), sampled[:, 1].max()) + 5.0

    for end in range(1, len(trace) + 1, frame_stride):
        fig, ax = plt.subplots(figsize=(7.0, 5.5))
        ax.plot(sampled[:, 0], sampled[:, 1], "k--", linewidth=1.6,
                label=labels("参考路径", "reference"))
        ax.plot(trace.x[:end], trace.y[:end], color="#1f77b4", linewidth=1.8,
                label=trace.controller)
        heading = trace.yaw[end - 1]
        ax.arrow(trace.x[end - 1], trace.y[end - 1],
                 2.2 * np.cos(heading), 2.2 * np.sin(heading),
                 head_width=0.8, head_length=1.0, color="#d62728", length_includes_head=True)
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_xlabel("x / m")
        ax.set_ylabel("y / m")
        ax.set_title(labels(f"Stanley 横向控制跟踪过程  t = {trace.time[end-1]:.1f} s",
                            f"Stanley tracking  t = {trace.time[end-1]:.1f} s"), fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper left", fontsize=8)
        ax.set_aspect("equal", adjustable="box")
        fig.tight_layout()
        fig.canvas.draw()
        buffer = np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()
        frames.append(buffer)
        plt.close(fig)

    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    imageio.mimsave(output, frames, fps=fps, loop=0)
    return output
