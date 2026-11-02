#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""可视化：轨迹对比、偏差曲线、指标柱状图。

matplotlib 是可选依赖：未安装时给出明确安装提示；`main.py` 加
``--no-figures`` 可在无绘图环境只跑数值。
"""

from __future__ import annotations

import os
from typing import Sequence

import numpy as np

from .simulation import Metrics, Trace


def _plt():
    try:
        import matplotlib
        matplotlib.use("Agg")          # 无显示器环境也能出图
        import matplotlib.pyplot as plt
        return plt
    except ImportError as exc:  # pragma: no cover - 环境相关
        raise RuntimeError("缺少 matplotlib，无法生成图表。请执行 pip3 install matplotlib，"
                           "或使用 --no-figures 跳过绘图。") from exc


def _setup_cjk(plt) -> bool:
    """尝试启用中文字体，避免图例/标题出现方块。"""
    from matplotlib import font_manager
    wanted = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "WenQuanYi Zen Hei"]
    have = {f.name for f in font_manager.fontManager.ttflist}
    for name in wanted:
        if name in have:
            plt.rcParams["font.sans-serif"] = [name]
            plt.rcParams["axes.unicode_minus"] = False
            return True
    return False


COLORS = {"pure_pursuit": "#1f77b4", "stanley": "#d62728", "lqr": "#2ca02c"}


def plot_trajectories(traces: Sequence[Trace], path, output: str) -> str:
    """参考路径与各控制器实际轨迹对比图。"""
    plt = _plt()
    cjk = _setup_cjk(plt)
    title = "横向控制轨迹对比" if cjk else "Lateral control trajectories"

    fig, ax = plt.subplots(figsize=(9, 7))
    sampled = path.sample(600)
    ax.plot(sampled[:, 0], sampled[:, 1], "k--", lw=2, label="参考路径" if cjk else "reference")
    for tr in traces:
        c = COLORS.get(tr.controller, None)
        ax.plot(tr.x, tr.y, color=c, lw=1.6, label=f"{tr.controller} ({tr.scenario})")
        ax.plot(tr.x[0], tr.y[0], "o", color=c, ms=5)
        ax.plot(tr.x[-1], tr.y[-1], "s", color=c, ms=5)
    ax.set_xlabel("x / m")
    ax.set_ylabel("y / m")
    ax.set_title(title)
    # 注意：这里刻意不使用 axis("equal")。参考轨迹的横向变化通常远小于纵向长度，
    # 等比例会让曲线被压成一条直线，反而看不清跟踪差异。
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    fig.savefig(output, dpi=130)
    plt.close(fig)
    return output


def plot_errors(traces: Sequence[Trace], output: str) -> str:
    """横向偏差与航向偏差随时间变化（双联图）。"""
    plt = _plt()
    cjk = _setup_cjk(plt)
    fig, axes = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    for tr in traces:
        c = COLORS.get(tr.controller, None)
        axes[0].plot(tr.t, tr.e, color=c, lw=1.3, label=tr.controller)
        axes[1].plot(tr.t, np.degrees(tr.psi), color=c, lw=1.3, label=tr.controller)
    axes[0].axhline(0, color="k", lw=0.8)
    for y in (0.2, -0.2):
        axes[0].axhline(y, color="gray", ls=":", lw=0.9)
    axes[0].set_ylabel("横向偏差 e / m" if cjk else "cross-track e / m")
    axes[0].set_title("横向偏差与航向偏差" if cjk else "Tracking errors")
    axes[0].grid(alpha=0.3)
    axes[0].legend(fontsize=9)
    axes[1].set_xlabel("时间 t / s" if cjk else "time t / s")
    axes[1].set_ylabel("航向偏差 / deg" if cjk else "heading err / deg")
    axes[1].grid(alpha=0.3)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    fig.savefig(output, dpi=130)
    plt.close(fig)
    return output


def plot_metrics(metrics: Sequence[Metrics], output: str) -> str:
    """四项关键指标的柱状图对比。

    横轴标签用「场景 + 控制器缩写」并旋转 30°：12 根柱子的完整名字（如
    pure_pursuit / stanley）会挤在一起互相重叠，图上就成了一团。
    """
    plt = _plt()
    cjk = _setup_cjk(plt)
    short = {"pure_pursuit": "PP", "stanley": "ST", "lqr": "LQR",
             "lookahead_pid": "PID"}
    names = [f"{m.scenario}\n{short.get(m.controller, m.controller)}" for m in metrics]
    panels = [
        ("平均绝对横向偏差 / m" if cjk else "mean |e| / m", [m.e_mean_abs for m in metrics]),
        ("最大横向偏差 / m" if cjk else "max |e| / m", [m.e_max_abs for m in metrics]),
        ("稳态横向偏差 / m" if cjk else "steady |e| / m", [m.e_steady_abs for m in metrics]),
        ("收敛时间 / s" if cjk else "convergence / s", [m.convergence_time for m in metrics]),
    ]
    colors = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e"]
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 7.5))
    for ax, (label, values), color in zip(axes.ravel(), panels, colors):
        pos = np.arange(len(values))
        ax.bar(pos, values, color=color, alpha=0.85)
        ax.set_xticks(pos)
        ax.set_xticklabels(names, fontsize=7.5, rotation=30, ha="right")
        ax.set_ylabel(label, fontsize=9)
        ax.grid(axis="y", alpha=0.3)
        top = max(values) if values else 1.0
        if top > 0:
            ax.set_ylim(0.0, top * 1.18)   # 给数值标注留出空间，避免贴顶
        for x, v in zip(pos, values):
            ax.text(x, v + top * 0.02, f"{v:.3f}", ha="center", va="bottom", fontsize=7)
    fig.suptitle("横向控制指标对比（横轴：场景 / 控制器，PP=纯跟踪，ST=Stanley）"
                 if cjk else "Lateral control metrics (scenario / controller)",
                 fontsize=11)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    fig.savefig(output, dpi=130)
    plt.close(fig)
    return output
