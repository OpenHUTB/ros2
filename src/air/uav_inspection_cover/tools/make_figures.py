#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate images/route_compare.png from the results JSON.

图全部来自实际运行：航线取结果 JSON 里的 waypoints，覆盖与航程数字
取同一份 JSON，不手工绘制任何示意曲线。
"""
from __future__ import annotations

import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
_AIR = os.path.dirname(_PKG)
for _p in (_PKG, os.path.join(_AIR, "uav_mujoco_sim")):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

OUT_NAME = "route_compare.png"


def figure_case(ax, mission):
    """单 case 面板：基线/改进航线 + 杆塔，英文标题、tab10+虚线主色。"""
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    for tw in _towers(mission):
        ax.plot(tw[0], tw[1], "^", color="0.35", ms=9)
    wp = np.asarray(mission["route_baseline"]["waypoints"])
    ax.plot(wp[:, 0], wp[:, 1], "--", color=colors[1], lw=1.2, alpha=0.7,
            label="baseline %.0f m" % mission["route_baseline"]["distance_m"])
    wp = np.asarray(mission["route_improved"]["waypoints"])
    ax.plot(wp[:, 0], wp[:, 1], "-", color=colors[0], lw=1.8,
            label="2-opt improved %.0f m"
            % mission["route_improved"]["distance_m"])
    ax.set_title("%s (%d towers, coverage %.0f%%)"
                 % (mission["case"], mission["towers"],
                    100.0 * mission["route_improved"]["coverage_rate"]))
    ax.set_xlabel("x / m")
    ax.set_ylabel("y / m")
    ax.grid(True, ls=":")
    ax.legend(fontsize=8, loc="lower right")
    ax.set_aspect("equal", adjustable="datalim")


def _towers(mission):
    """从航线 JSON 反推塔位不可行，直接重算场景（确定性的）。"""
    from uav_inspection_cover.core.planner import build_cases
    return [(t.x, t.y) for t in build_cases()[mission["case"]].towers]


def figure_bars(ax, missions):
    """汇总面板：改进前后航程与覆盖率对比。"""
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    idx = np.arange(len(missions))
    names = [m["case"].replace("case_grid_", "") for m in missions]
    base = [m["route_baseline"]["distance_m"] for m in missions]
    impr = [m["route_improved"]["distance_m"] for m in missions]
    ax.bar(idx - 0.2, base, 0.4, color=colors[1], label="baseline")
    ax.bar(idx + 0.2, impr, 0.4, color=colors[0], label="2-opt improved")
    for i, (b, v) in enumerate(zip(base, impr)):
        ax.text(i - 0.2, b + 20, "%.0f" % b, ha="center", fontsize=8)
        ax.text(i + 0.2, v + 20, "%.0f" % v, ha="center", fontsize=8)
    ax.set_xticks(idx, names)
    ax.set_ylabel("route distance / m")
    ax.set_title("Route distance: baseline vs 2-opt improved")
    ax.grid(True, axis="y", ls=":")
    ax.legend(fontsize=8, loc="lower right")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="generate route_compare.png")
    ap.add_argument("--json",
                    default=os.path.join(_PKG, "results",
                                         "inspection_result.json"))
    ap.add_argument("--out-dir", default=os.path.join(_PKG, "images"))
    args = ap.parse_args(argv)

    with open(args.json, encoding="utf-8") as fh:
        missions = json.load(fh)["missions"]

    os.makedirs(args.out_dir, exist_ok=True)
    fig, axes = plt.subplots(1, len(missions) + 1,
                             figsize=(4.6 * (len(missions) + 1), 4.4))
    for ax, m in zip(axes[:-1], missions):
        figure_case(ax, m)
    figure_bars(axes[-1], missions)
    fig.suptitle("Inspection route comparison (baseline vs 2-opt)", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    out = os.path.join(args.out_dir, OUT_NAME)
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print("[figures] written %s from %s" % (out, args.json))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
