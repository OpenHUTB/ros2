#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从运行结果 JSON 生成文档用图：甘特图（images/gantt.png）与
贪心 vs 改进对比图（images/compare.png）。

只读 ``results/all_results.json``（``python main.py`` 自动落盘），
不重跑算法与仿真，保证图里的数字与 JSON 一致。
"""
from __future__ import annotations

import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
for _p in (_PKG,):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from matplotlib import font_manager  # noqa: E402

for _name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "DejaVu Sans"):
    if any(f.name == _name for f in font_manager.fontManager.ttflist):
        plt.rcParams["font.sans-serif"] = [_name]
        break
plt.rcParams["axes.unicode_minus"] = False

COLORS = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e"]
OUT_DIR = os.path.join(_PKG, "images")


def load_results(path: str) -> list:
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def figure_gantt(results: list, path: str) -> str:
    """每个场景两行子图：贪心（上）vs 改进后（下）的机-任务甘特图。

    色块 = 任务占用区间（到达 → 服务结束），深色描边 = 服务开始时刻
    相对时间窗的状态（绿=准点、蓝=早到等待、红=晚到）。
    """
    n = len(results)
    fig, axes = plt.subplots(n, 2, figsize=(13.5, 2.6 * n + 1.2),
                             squeeze=False)
    for row, r in enumerate(results):
        uids = sorted({row_["uid"] for row_ in r["per_uav"]})
        y_of = {uid: len(uids) - 1 - i for i, uid in enumerate(uids)}
        stops_of = {row_["uid"]: row_["stops"] for row_ in r["per_uav"]}

        for col, (key, title) in enumerate(
                (("greedy", "贪心（EDF）"), ("final", "改进后"))):
            ax = axes[row][col]
            # JSON 里逐机时刻表在 per_uav（改进后）；贪心侧由 stops 重放
            if key == "final":
                source = {row_["uid"]: row_["stops"] for row_ in r["per_uav"]}
            else:
                source = _replay_greedy(r)
            for uid, stops in source.items():
                for st in stops:
                    y = y_of[uid]
                    status = st["status"]
                    face = {"early": "#aec7e8", "on-time": "#98df8a",
                            "late": "#ff9896"}[status]
                    start = st["arrival"]
                    width = max(st["departure"] - start, 0.5)
                    ax.broken_barh([(start, width)], (y - 0.35, 0.7),
                                   facecolors=face, edgecolors="k", lw=0.6)
                    ax.text(start + width / 2, y,
                            st["tid"].split("-", 1)[-1],
                            ha="center", va="center", fontsize=7)
                    # 服务开始时刻标记
                    ax.plot(st["serve_start"], y + 0.35, "kv", ms=4)
            for i, uid in enumerate(uids):
                ax.axhline(i + 0.5, color="#cccccc", lw=0.6)
            ax.set_yticks(range(len(uids)))
            ax.set_yticklabels(list(reversed(uids)))
            ax.set_xlim(0, max(60.0, max(
                st["departure"] for stops in source.values() for st in stops)
                * 1.05))
            ax.set_ylim(-0.6, len(uids) - 0.4)
            delay = r[key]["delay_total"] if "delay_total" in r[key] \
                else r[key]["lateness_total"]
            ax.set_title("(%s) %s：总延误 %.1f s"
                         % ("ab"[col], title, delay), fontsize=10)
            ax.grid(True, axis="x", ls=":")
            if row == n - 1:
                ax.set_xlabel("时间 / s")
        axes[row][0].set_ylabel("%s\n%s" % (r["scenario"], r["title"]),
                                fontsize=10)
    # 图例
    import matplotlib.patches as mpatches
    handles = [mpatches.Patch(facecolor="#aec7e8", edgecolor="k", label="早到等待"),
               mpatches.Patch(facecolor="#98df8a", edgecolor="k", label="准点"),
               mpatches.Patch(facecolor="#ff9896", edgecolor="k", label="晚到")]
    fig.legend(handles=handles, loc="upper center", ncol=3,
               frameon=False, bbox_to_anchor=(0.5, 1.0))
    fig.tight_layout(rect=(0, 0, 1, 0.965))
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def _replay_greedy(r: dict) -> dict:
    """从 JSON 重放贪心分配的时刻表（与落盘时同参数）。"""
    sys.path.insert(0, _PKG)
    from uav_time_window.core.mission import Config, Task, Uav
    from uav_time_window.core.window import schedule_uav

    cfg = Config(lambda_balance=r["allocation"].get(
        "lambda_balance", 0.5))
    uavs = {u["uid"]: Uav(u["uid"], u["x"], u["y"], u["speed"],
                          u["endurance"]) for u in r["uavs"]}
    tasks = {t["tid"]: Task(t["tid"], t["x"], t["y"], t["earliest"],
                            t["latest"], t["service_time"], t["priority"])
             for t in r["tasks"]}
    out = {}
    for uid, tids in r["allocation"]["greedy_assign"].items():
        sched = schedule_uav(uavs[uid], tids, tasks, cfg)
        out[uid] = [s.as_dict() for s in sched.stops]
    return out


def figure_compare(results: list, path: str) -> str:
    """贪心 vs 改进：总延误与最大单机延误对比柱状图。"""
    import numpy as np
    idx = np.arange(len(results))
    names = [r["scenario"] for r in results]
    g_total = [r["greedy"]["delay_total"] for r in results]
    f_total = [r["final"]["delay_total"] for r in results]
    g_max = [r["greedy"]["delay_max_uav"] for r in results]
    f_max = [r["final"]["delay_max_uav"] for r in results]

    fig, axes = plt.subplots(1, 2, figsize=(12.0, 4.2))
    ax = axes[0]
    ax.bar(idx - 0.2, g_total, 0.4, label="贪心（EDF）", color="#d62728")
    ax.bar(idx + 0.2, f_total, 0.4, label="改进后", color="#2ca02c")
    for i, v in enumerate(f_total):
        ax.text(i + 0.2, v + 0.5, "%.1f" % v, ha="center", fontsize=8)
    ax.set_xticks(idx, names)
    ax.set_ylabel("总正延误 / s")
    ax.set_title("(a) 总延误对比")
    ax.grid(True, axis="y", ls=":")
    ax.legend(fontsize=9)

    ax = axes[1]
    ax.bar(idx - 0.2, g_max, 0.4, label="贪心（EDF）", color="#d62728")
    ax.bar(idx + 0.2, f_max, 0.4, label="改进后", color="#2ca02c")
    for i, v in enumerate(f_max):
        ax.text(i + 0.2, v + 0.3, "%.1f" % v, ha="center", fontsize=8)
    ax.set_xticks(idx, names)
    ax.set_ylabel("最大单机延误 / s")
    ax.set_title("(b) 最大单机延误对比（max）")
    ax.grid(True, axis="y", ls=":")
    ax.legend(fontsize=9)

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="从结果 JSON 生成配图")
    ap.add_argument("--json", default=os.path.join(_PKG, "results",
                                                   "all_results.json"))
    ap.add_argument("--out-dir", default=OUT_DIR)
    args = ap.parse_args(argv)
    os.makedirs(args.out_dir, exist_ok=True)

    results = load_results(args.json)
    p1 = figure_gantt(results, os.path.join(args.out_dir, "gantt.png"))
    print("已生成", p1)
    p2 = figure_compare(results, os.path.join(args.out_dir, "compare.png"))
    print("已生成", p2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
