#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""结果可视化：噪声足迹 45/55/65 dB 等值线（优化前后并排）。

绘图不属于 core（core 只依赖 numpy）；本模块只被 cli 与 tools/make_figures
调用，matplotlib 固定 Agg 后端，中文字体按 SPEC 要求配置。
"""
from __future__ import annotations

import os
from typing import Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import font_manager  # noqa: E402

from .core.acoustic import DEFAULT  # noqa: E402
from .core.exposure import contour_segments, grid_footprint  # noqa: E402
from .core.pipeline import CONTOUR_LEVELS, CaseResult  # noqa: E402
from .core.track import Flight  # noqa: E402

for _name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "DejaVu Sans"):
    if any(f.name == _name for f in font_manager.fontManager.ttflist):
        plt.rcParams["font.sans-serif"] = [_name]
        break
plt.rcParams["axes.unicode_minus"] = False

LEVEL_STYLE = {45.0: ("#7fbf7f", "--"), 55.0: ("#ff9f40", "-"),
               65.0: ("#d62728", "-")}


def _draw_panel(ax, flights: Sequence[Flight], points, extent, title: str,
                ds: float = 2.0) -> None:
    """一个面板：LAeq 场 + 等值线 + 航迹 + 敏感点。"""
    grid, _ = grid_footprint(flights, ds=ds, extent=extent)
    xmin, xmax, ymin, ymax = extent
    ax.imshow(grid, origin="lower", extent=(xmin, xmax, ymin, ymax),
              cmap="viridis", vmin=30.0, vmax=75.0, aspect="equal",
              interpolation="bilinear")
    for lv in CONTOUR_LEVELS:
        color, ls = LEVEL_STYLE[lv]
        segs = contour_segments(grid, lv, extent)
        for (a, b) in segs:
            ax.plot([a[0], b[0]], [a[1], b[1]], color=color, ls=ls, lw=1.0)
    for i, f in enumerate(flights):
        xs = [p[0] for p in f.path]
        ys = [p[1] for p in f.path]
        ax.plot(xs, ys, "-", color="#ffffff", lw=1.6, alpha=0.9)
        ax.plot(xs[0], ys[0], "s", color="#00d5ff", ms=7, zorder=5)
        ax.plot(xs[-1], ys[-1], "^", color="#00d5ff", ms=7, zorder=5)
        ax.annotate("%s %.0fm/%.0f°" % (f.uid, f.alt, f.climb_angle),
                    (xs[len(xs) // 2], ys[len(ys) // 2]),
                    color="#ffffff", fontsize=8,
                    ha="center", va="bottom")
    for pt in points:
        ax.plot(pt.x, pt.y, "p", color="#ff4fa0", ms=11, zorder=6)
        ax.annotate(pt.name, (pt.x, pt.y), color="#ff4fa0", fontsize=9,
                    xytext=(4, 4), textcoords="offset points", zorder=6)
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_title(title, fontsize=11)
    ax.set_xlabel("x 东向 (m)")
    ax.set_ylabel("y 北向 (m)")


def draw_footprint_all(results: Sequence[CaseResult], out_dir: str) -> str:
    """所有场景：两行（优化前/后）× N 列（场景），存 images/footprint.png。"""
    n = len(results)
    fig, axes = plt.subplots(2, n, figsize=(7.2 * n, 9.0), squeeze=False)
    for k, case in enumerate(results):
        s = case.summary
        extent = tuple(s["footprint"]["extent"])
        base = [Flight.from_dict(d) for d in s["baseline_flights"]]
        opt = [Flight.from_dict(d) for d in s["optimized_flights"]]
        _draw_panel(axes[0][k], base, case.scenario.points, extent,
                    "%s：优化前（加权目标 %.1f dB）"
                    % (case.name, s["objective_before"]))
        _draw_panel(axes[1][k], opt, case.scenario.points, extent,
                    "优化后（%.1f dB，Δ%.1f dB）"
                    % (s["objective_after"], s["improvement_db"]))
    handles = [plt.Line2D([], [], color=LEVEL_STYLE[lv][0], ls=LEVEL_STYLE[lv][1],
                          label="%d dB" % lv) for lv in CONTOUR_LEVELS]
    handles.append(plt.Line2D([], [], color="#ff4fa0", marker="p", ls="",
                              label="敏感点"))
    fig.legend(handles=handles, loc="lower center", ncol=4, fontsize=10)
    fig.suptitle("噪声足迹等值线（LAeq 场，优化前 / 后）", fontsize=14)
    fig.tight_layout(rect=(0, 0.05, 1, 0.97))
    path = os.path.join(out_dir, "footprint.png")
    fig.savefig(path, dpi=130)
    plt.close(fig)

    # 每场景单独一张（文档引用用）
    for case in results:
        s = case.summary
        extent = tuple(s["footprint"]["extent"])
        fig, axes = plt.subplots(1, 2, figsize=(14.4, 6.0))
        base = [Flight.from_dict(d) for d in s["baseline_flights"]]
        opt = [Flight.from_dict(d) for d in s["optimized_flights"]]
        _draw_panel(axes[0], base, case.scenario.points, extent,
                    "优化前（加权目标 %.1f dB）" % s["objective_before"])
        _draw_panel(axes[1], opt, case.scenario.points, extent,
                    "优化后（%.1f dB，Δ%.1f dB）"
                    % (s["objective_after"], s["improvement_db"]))
        fig.legend(handles=handles, loc="lower center", ncol=4, fontsize=10)
        fig.suptitle("%s 场景噪声足迹（优化前 / 后并排）" % case.name,
                     fontsize=14)
        fig.tight_layout(rect=(0, 0.06, 1, 0.95))
        fig.savefig(os.path.join(out_dir, "footprint_%s.png" % case.name),
                    dpi=130)
        plt.close(fig)
    return path
