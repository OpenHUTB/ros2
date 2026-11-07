#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Make docs figure images/batches.png from results/*.json (matplotlib Agg).

单行英文摘要 + 中文正文。
左列三个 case 的架次划分图（配送中心 + 各站按架次着色连线），右列各架次
计划航程对比柱状图。所有数字来自实际运行的 results JSON，不是手绘示意。
B 套风格：tab10 配色 + 虚线主色、dpi=110、英文标题、图例 lower right。
"""
from __future__ import annotations

import argparse
import glob
import json
import os

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

if _PKG not in __import__("sys").path:
    __import__("sys").path.insert(0, _PKG)
from uav_delivery_route.core.cases import get_case  # noqa: E402

plt.rcParams["axes.unicode_minus"] = False
OUT_PATH = os.path.join(_PKG, "images", "batches.png")


def main(results_dir: str = os.path.join(_PKG, "results"),
         out_path: str = OUT_PATH) -> str:
    paths = sorted(glob.glob(os.path.join(results_dir, "*_result.json")))
    if not paths:
        raise SystemExit("no *_result.json under %s; run main.py first" % results_dir)
    from uav_delivery_route.core.cases import CASE_NAMES
    order = {n: i for i, n in enumerate(CASE_NAMES)}
    results = []
    for p in paths:
        with open(p, encoding="utf-8") as f:
            results.append(json.load(f))
    results.sort(key=lambda r: order.get(r["case"], 99))

    fig, axes = plt.subplots(2, len(results),
                             figsize=(5.2 * len(results), 9.0),
                             gridspec_kw=dict(height_ratios=[2.1, 1.0]))
    if len(results) == 1:
        axes = [[axes[0]], [axes[1]]]
    cmap = plt.get_cmap("tab10")

    for col, r in enumerate(results):
        plan = r["plan"]
        depot = (0.0, 0.0)
        ax = axes[0][col]
        # 站点坐标统一从 core.cases 读取（与结果 JSON 同一数据源）
        case = get_case(r["case"])
        pos = {s.sid: (s.x, s.y) for s in case.stations}
        for k, so in enumerate(plan["sorties"]):
            xs = [depot[0]] + [pos[sid][0] for sid in so["stations"]] + [depot[0]]
            ys = [depot[1]] + [pos[sid][1] for sid in so["stations"]] + [depot[1]]
            c = cmap(k % 10)
            ax.plot(xs, ys, "--", color=c, lw=1.8, alpha=0.9,
                    label="sortie %d (%.0f m)" % (k, so["distance_m"]))
            ax.plot([x for x in xs[1:-1]], [y for y in ys[1:-1]], "o",
                    color=c, ms=5)
        ax.plot(depot[0], depot[1], "*", color="k", ms=14, label="depot")
        ax.set_title("%s: %d sorties / %.0f m" %
                     (r["case"], plan["sortie_count"], plan["total_distance_m"]),
                     fontsize=11)
        ax.set_xlabel("x east / m")
        ax.set_ylabel("y north / m")
        ax.grid(True, ls=":")
        ax.legend(fontsize=7.5, loc="lower right")
        ax.set_aspect("equal", adjustable="datalim")

        # 右下：各架次计划航程对比
        ax2 = axes[1][col]
        ks = [so["index"] for so in plan["sorties"]]
        ds = [so["distance_m"] for so in plan["sorties"]]
        ax2.bar(ks, ds, color=[cmap(k % 10) for k in ks], alpha=0.85)
        ax2.set_title("Planned distance per sortie", fontsize=11)
        ax2.set_xlabel("sortie index")
        ax2.set_ylabel("distance / m")
        ax2.set_xticks(ks)
        ax2.grid(True, ls=":", axis="y")
        ax2.legend(handles=[], loc="lower right")

    fig.suptitle("UAV delivery batch partition and per-sortie route length",
                 fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    fig.savefig(out_path, dpi=110)
    print("[figures] written %s" % out_path)
    return out_path


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", default=os.path.join(_PKG, "results"))
    ap.add_argument("--out", default=OUT_PATH)
    a = ap.parse_args()
    main(a.results, a.out)
