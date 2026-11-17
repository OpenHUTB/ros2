#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成文档配图：态势对比、真飞轨迹、间隔时间曲线、场景汇总。

图里的每一条曲线都来自实际运行：计划航迹来自 ``Plan``，真飞轨迹与间隔
来自 MuJoCo 闭环 ``run_fleet`` 的输出。没有手工绘制的示意曲线。
"""
from __future__ import annotations

import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
_AIR = os.path.dirname(_PKG)
_REPO = os.path.dirname(os.path.dirname(_AIR))
for _p in (_PKG, os.path.join(_AIR, "uav_mujoco_sim")):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import font_manager  # noqa: E402

from uav_deconflict.core.conflict import detect  # noqa: E402
from uav_deconflict.core.metrics import truth_separation_series  # noqa: E402
from uav_deconflict.core.resolve import report_rows  # noqa: E402

for _name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "DejaVu Sans"):
    if any(f.name == _name for f in font_manager.fontManager.ttflist):
        plt.rcParams["font.sans-serif"] = [_name]
        break
plt.rcParams["axes.unicode_minus"] = False

COLORS = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd", "#8c564b",
          "#17becf", "#bcbd22"]
OUT_DIR = os.path.join(_REPO, "docs", "img", "air")


def _xy(plan):
    xs = [p[0] for p in plan.path]
    ys = [p[1] for p in plan.path]
    return xs, ys


def figure_case(result, path):
    sc = result.scenario
    fig, axes = plt.subplots(2, 2, figsize=(13.2, 10.0))
    ax = axes[0][0]
    cons = detect(result.baseline, sc.sep)
    for i, plan in enumerate(result.baseline):
        xs, ys = _xy(plan)
        ax.plot(xs, ys, "-o", color=COLORS[i % len(COLORS)], lw=1.8, ms=4,
                label="%s %s (%.0f m)" % (plan.uid, plan.label, plan.alt))
        ax.plot(xs[0], ys[0], "s", color=COLORS[i % len(COLORS)], ms=7)
    for c in cons:
        pa = next(p for p in result.baseline if p.uid == c.a)
        pb = next(p for p in result.baseline if p.uid == c.b)
        ca, cb = pa.position_at(c.cpa_t), pb.position_at(c.cpa_t)
        if ca is not None and cb is not None:
            mx, my = (ca[0] + cb[0]) / 2, (ca[1] + cb[1]) / 2
            ax.plot(mx, my, "x", color="k", ms=9, mew=2.2)
    ax.set_title("(a) 解脱前计划航迹：%d 段冲突（× 为最近接近点）" % len(cons))
    ax.set_xlabel("x 东 / m")
    ax.set_ylabel("y 北 / m")
    ax.grid(True, ls=":")
    ax.legend(fontsize=7.5, loc="best")
    ax.set_aspect("equal", adjustable="datalim")

    ax = axes[0][1]
    for i, plan in enumerate(result.baseline):
        xs, ys = _xy(plan)
        ax.plot(xs, ys, "--", color=COLORS[i % len(COLORS)], lw=1.2, alpha=0.55)
    for i, plan in enumerate(result.baseline):
        rp = result.resolution.plans[plan.uid]
        xs, ys = _xy(rp)
        ax.plot(xs, ys, "-o", color=COLORS[i % len(COLORS)], lw=2.0, ms=4,
                label="%s 高%.0f m 延误%.0fs"
                      % (rp.uid, rp.alt, rp.end_time - plan.end_time))
    ax.set_title("(b) 解脱后计划（虚线为原航路）")
    ax.set_xlabel("x 东 / m")
    ax.set_ylabel("y 北 / m")
    ax.grid(True, ls=":")
    ax.legend(fontsize=7.5, loc="best")
    ax.set_aspect("equal", adjustable="datalim")

    ax = axes[1][0]
    for i, uid in enumerate(sorted(result.sim_after.pos)):
        pos = result.sim_after.pos[uid]
        ax.plot(pos[:, 0], pos[:, 1], "-", color=COLORS[i % len(COLORS)], lw=1.8,
                label="%s 真飞" % uid)
    ax.set_title("(c) MuJoCo 闭环真飞轨迹（%d 架）" % len(result.sim_after.pos))
    ax.set_xlabel("x 东 / m")
    ax.set_ylabel("y 北 / m")
    ax.grid(True, ls=":")
    ax.legend(fontsize=8, loc="best")
    ax.set_aspect("equal", adjustable="datalim")

    ax = axes[1][1]
    t1, h1 = truth_separation_series(result.sim_before, sc.sep)
    t2, h2 = truth_separation_series(result.sim_after, sc.sep)
    ax.plot(t1, np.where(np.isfinite(h1), h1, np.nan), color="#d62728", lw=1.6,
            label="解脱前（真飞）")
    ax.plot(t2, np.where(np.isfinite(h2), h2, np.nan), color="#2ca02c", lw=1.8,
            label="解脱后（真飞）")
    ax.axhline(sc.sep.horizontal, color="k", ls="--", lw=1.2,
               label="水平间隔标准 %.0f m" % sc.sep.horizontal)
    finite = np.concatenate([h1[np.isfinite(h1)], h2[np.isfinite(h2)]])
    ax.set_ylim(0, min(160.0, max(90.0, float(np.nanmax(finite)) * 1.15)))
    ax.set_xlabel("时间 / s")
    ax.set_ylabel("机队最小水平间隔 / m")
    ax.set_title("(d) 真飞最小间隔（垂直 < %.0f m 时统计）" % sc.sep.vertical)
    ax.grid(True, ls=":")
    ax.legend(fontsize=8, loc="lower right")

    fig.suptitle("%s — %s" % (sc.name, sc.title), fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def figure_summary(results, path):
    rows = []
    for r in results:
        s = r.summary
        rows.append(dict(
            name=r.name,
            before=s["before"]["conflicts"],
            after=s["after"]["conflicts"],
            delay=s["delay_total"],
            truth=s.get("truth_after", {}).get("min_h", np.nan),
            truth_before=s.get("truth_before", {}).get("min_h", np.nan),
        ))

    fig, axes = plt.subplots(1, 3, figsize=(14.0, 4.2))
    idx = np.arange(len(rows))
    names = [r["name"] for r in rows]

    ax = axes[0]
    ax.bar(idx - 0.2, [r["before"] for r in rows], 0.4, label="解脱前",
           color="#d62728")
    ax.bar(idx + 0.2, [r["after"] for r in rows], 0.4, label="解脱后",
           color="#2ca02c")
    ax.set_xticks(idx, names)
    ax.set_ylabel("冲突段数")
    ax.set_title("(a) 冲突消解")
    ax.grid(True, axis="y", ls=":")
    ax.legend(fontsize=9)

    ax = axes[1]
    ax.bar(idx, [r["delay"] for r in rows], 0.5, color="#1f77b4")
    for i, r in enumerate(rows):
        ax.text(i, r["delay"] + 1.0, "%.1f s" % r["delay"], ha="center",
                fontsize=8)
    ax.set_xticks(idx, names)
    ax.set_ylabel("总延误 / s")
    ax.set_title("(b) 解脱代价")
    ax.grid(True, axis="y", ls=":")

    ax = axes[2]
    ax.bar(idx - 0.2, [r["truth_before"] for r in rows], 0.4, label="解脱前",
           color="#d62728")
    ax.bar(idx + 0.2, [r["truth"] for r in rows], 0.4, label="解脱后",
           color="#2ca02c")
    ax.axhline(50.0, color="k", ls="--", lw=1.2, label="标准 50 m")
    ax.set_xticks(idx, names)
    ax.set_ylabel("真飞最小水平间隔 / m")
    ax.set_title("(c) 闭环真飞间隔")
    ax.grid(True, axis="y", ls=":")
    ax.legend(fontsize=9)

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def main(argv=None):
    ap = argparse.ArgumentParser(description="生成配图")
    ap.add_argument("--scenario", default="metro",
                    help="用于态势对比图的场景")
    ap.add_argument("--out-dir", default=OUT_DIR)
    ap.add_argument("--json", default="", help="同时落盘机读结果")
    args = ap.parse_args(argv)

    os.makedirs(args.out_dir, exist_ok=True)
    from uav_deconflict.core.pipeline import run_all, to_json

    # 一次跑完全部场景，配图与 JSON 用同一份结果，避免数字对不上
    print("运行全部场景（含 MuJoCo 闭环）...")
    results = run_all(["crossing", "corridor", "metro", "slot"], sim=True,
                      progress=True)
    case = next(r for r in results if r.name == args.scenario)

    p1 = os.path.join(args.out_dir, "uav_deconflict_%s.png" % args.scenario)
    figure_case(case, p1)
    print("已生成", p1)

    p2 = os.path.join(args.out_dir, "uav_deconflict_summary.png")
    figure_summary(results, p2)
    print("已生成", p2)

    if args.json:
        print("机读结果已写入", to_json(results, args.json))

    print("%-10s %6s %6s %10s %12s" % ("场景", "前", "后", "总延误s", "真飞间隔m"))
    for r in results:
        s = r.summary
        print("%-10s %6d %6d %10.1f %12.1f" %
              (r.name, s["before"]["conflicts"], s["after"]["conflicts"],
               s["delay_total"], s.get("truth_after", {}).get("min_h", float("nan"))))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
