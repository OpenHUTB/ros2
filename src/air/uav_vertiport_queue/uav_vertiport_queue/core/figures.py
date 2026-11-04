#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""结果配图：容量-请求率曲线与排队前后对比（matplotlib，Agg 后端）。"""
from __future__ import annotations

import os
from typing import Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import font_manager  # noqa: E402

for _name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "DejaVu Sans"):
    if any(f.name == _name for f in font_manager.fontManager.ttflist):
        plt.rcParams["font.sans-serif"] = [_name]
        break
plt.rcParams["axes.unicode_minus"] = False


def capacity_figure(scan: Sequence[dict], saturation: dict,
                    path: str, wait_threshold: float = 60.0) -> str:
    """容量-请求率曲线：吞吐量与平均等待（含饱和点标记）。"""
    rates = [row["rate"] for row in scan]
    thr = [row["throughput"] for row in scan]
    wait = [row["avg_wait"] for row in scan]

    fig, axes = plt.subplots(1, 2, figsize=(12.5, 4.6))
    ax = axes[0]
    ax.plot(rates, rates, "--", color="#999999", lw=1.2, label="理想吞吐（无排队）")
    ax.plot(rates, thr, "-o", color="#1f77b4", lw=2.0, ms=4, label="实际吞吐")
    if saturation.get("rate"):
        ax.axvline(saturation["rate"], color="#d62728", ls=":", lw=1.5,
                   label="饱和点 %.0f 架次/h" % saturation["rate"])
    ax.set_xlabel("请求率 /（架次/h）")
    ax.set_ylabel("吞吐量 /（架次/h）")
    ax.set_title("(a) 吞吐量-请求率曲线")
    ax.grid(True, ls=":")
    ax.legend(fontsize=9, loc="upper left")

    ax = axes[1]
    ax.plot(rates, wait, "-o", color="#2ca02c", lw=2.0, ms=4, label="平均等待")
    ax.axhline(wait_threshold, color="#d62728", ls="--", lw=1.2,
               label="等待阈值 %d s" % int(wait_threshold))
    if saturation.get("rate"):
        ax.axvline(saturation["rate"], color="#d62728", ls=":", lw=1.5)
    ax.set_xlabel("请求率 /（架次/h）")
    ax.set_ylabel("平均等待 / s")
    ax.set_title("(b) 平均等待-请求率曲线")
    ax.grid(True, ls=":")
    ax.legend(fontsize=9, loc="upper left")

    fig.tight_layout()
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def wait_figure(results: Sequence[object], path: str) -> str:
    """各场景排队前 vs 后：平均等待与起降位冲突对数。"""
    names = [r.scenario.name for r in results]
    avg_b = [r.metrics_before["avg_wait"] for r in results]
    avg_a = [r.metrics_after["avg_wait"] for r in results]
    con_b = [r.metrics_before["pad_conflicts"] for r in results]
    con_a = [r.metrics_after["pad_conflicts"] for r in results]

    fig, axes = plt.subplots(1, 2, figsize=(12.5, 4.4))
    idx = np.arange(len(names))
    ax = axes[0]
    ax.bar(idx - 0.2, avg_b, 0.4, label="排队前", color="#d62728")
    ax.bar(idx + 0.2, avg_a, 0.4, label="排队后", color="#2ca02c")
    ax.set_xticks(idx, names)
    ax.set_ylabel("平均等待 / s")
    ax.set_title("(a) 平均等待")
    ax.grid(True, axis="y", ls=":")
    ax.legend(fontsize=9)

    ax = axes[1]
    ax.bar(idx - 0.2, con_b, 0.4, label="排队前", color="#d62728")
    ax.bar(idx + 0.2, con_a, 0.4, label="排队后", color="#2ca02c")
    ax.set_xticks(idx, names)
    ax.set_ylabel("起降位占用冲突对数")
    ax.set_title("(b) 起降位占用冲突")
    ax.grid(True, axis="y", ls=":")
    ax.legend(fontsize=9)

    fig.tight_layout()
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path
