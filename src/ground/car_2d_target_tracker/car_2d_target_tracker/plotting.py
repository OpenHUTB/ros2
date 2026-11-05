#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""可选绘图：轨迹对比图 + 指标对比柱状图（matplotlib 缺失时整体跳过）。

坐标轴与图例用英文（matplotlib 默认字体没有中文字形）。
"""
from __future__ import annotations

import os

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


def plot_trajectories(results, save_dir, cfg):
    """三个场景的轨迹对比：上排 baseline、下排 escape。"""
    plt = _plt()
    names = list(results.keys())
    fig, axes = plt.subplots(len(names), 2, figsize=(11.0, 4.2 * len(names)), dpi=FIG_DPI)
    if len(names) == 1:
        axes = axes.reshape(1, 2)

    for row, name in enumerate(names):
        for col, mode in enumerate(("baseline", "escape")):
            ax = axes[row][col]
            series, m = results[name][mode]
            for (ox, oy, r) in series["obstacles"]:
                ax.add_patch(plt.Circle((ox, oy), r, color="#8a8a95", alpha=0.55))
            ax.plot(series["x"], series["y"], color="#2f8f4e", linewidth=1.6,
                    label="car path")
            ax.plot(series["x"][-1], series["y"][-1], "o", color="#1f77b4",
                    markersize=6, label="car end")
            tx, ty = series["target_track"]
            ax.plot(tx, ty, "o", color="#d62728", markersize=8, label="target")
            ax.set_title("%s / %s  reached=%s arrive=%.2fs" %
                         (name, mode, m["reached"],
                          m["arrive_time"] if m["reached"] else float("nan")), fontsize=9)
            ax.set_xlim(0, cfg.width)
            ax.set_ylim(0, cfg.height)
            ax.set_aspect("equal")
            ax.grid(alpha=0.25)
            ax.legend(loc="upper right", fontsize=7)
    fig.tight_layout()
    path = os.path.join(save_dir, "tracker_trajectories.png")
    fig.savefig(path, dpi=FIG_DPI)
    plt.close(fig)
    return path


def plot_metrics(results, save_dir):
    """到达时间与卡死占比的对照柱状图。"""
    plt = _plt()
    names = list(results.keys())
    fig, axes = plt.subplots(1, 2, figsize=(11.0, 4.0), dpi=FIG_DPI)

    x = range(len(names))
    width = 0.36
    for i, mode in enumerate(("baseline", "escape")):
        arrive = [results[n][mode][1]["arrive_time"] for n in names]
        arrive = [v if v == v else 0.0 for v in arrive]
        stuck = [results[n][mode][1]["stuck_fraction"] * 100.0 for n in names]
        xs = [p + (i - 0.5) * width for p in x]
        axes[0].bar(xs, arrive, width=width, label=mode)
        axes[1].bar(xs, stuck, width=width, label=mode)

    for ax, title, ylabel in ((axes[0], "time to reach target", "arrive time (s)"),
                              (axes[1], "stuck fraction before arrival", "stuck time (%)")):
        ax.set_xticks(list(x))
        ax.set_xticklabels(names)
        ax.set_title(title, fontsize=9)
        ax.set_ylabel(ylabel)
        ax.grid(alpha=0.25, axis="y")
        ax.legend(fontsize=8)

    fig.tight_layout()
    path = os.path.join(save_dir, "tracker_metrics.png")
    fig.savefig(path, dpi=FIG_DPI)
    plt.close(fig)
    return path


def export_all(results, save_dir, cfg):
    if not matplotlib_available():
        print("[提示] 未安装 matplotlib，跳过 PNG 绘图（CSV 已导出，不影响结果）。")
        print("       需要出图时：pip install matplotlib")
        return []
    os.makedirs(save_dir, exist_ok=True)
    return [plot_trajectories(results, save_dir, cfg),
            plot_metrics(results, save_dir)]
