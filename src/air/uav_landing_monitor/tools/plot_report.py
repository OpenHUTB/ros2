#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把监控 CSV 画成"降落过程证据图"（论文/PR 用）。

    python3 tools/plot_report.py landing.csv --out landing.png --title "正常降落"

上图：高度-时间曲线 + 目标高度虚线；下图：垂直速度指令，并把告警画成竖线标注。
CSV 由节点参数 ``monitor/csv`` 生成，列为 ``t,phase,z,target_z,vz_cmd,status,alerts``
（``alerts`` 里多条用 ``|`` 分隔）。
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

LEVEL_OF = {
    "INPUT_STALE": "error",
    "CMD_NO_EFFECT": "error",
    "DESCEND_STALL": "warn",
    "FREE_FALL": "error",
    "OVERSHOOT": "error",
    "LANDING_TIMEOUT": "error",
    "LANDED": "ok",
}
COLOR_OF = {"error": "#d62728", "warn": "#ff7f0e", "ok": "#2ca02c"}


def load(path):
    ts, zs, targets, vzs, codes = [], [], [], [], []
    with open(path, "r", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            try:
                ts.append(float(row["t"]))
                zs.append(float(row["z"]))
                vzs.append(float(row["vz_cmd"]))
                targets.append(float(row.get("target_z", "nan")))
            except (KeyError, TypeError, ValueError):
                continue
            raw = (row.get("alerts") or "").strip()
            codes.append([c for c in raw.split("|") if c])
    return ts, zs, targets, vzs, codes


def main(argv=None):
    ap = argparse.ArgumentParser(description="降落过程监控 CSV -> 证据图")
    ap.add_argument("csv", help="monitor/csv 生成的 CSV")
    ap.add_argument("--out", default="landing_monitor.png")
    ap.add_argument("--title", default="")
    ap.add_argument("--dpi", type=int, default=150)
    args = ap.parse_args(argv)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # 中文字体：按平台常见字体依次尝试，避免中文变成方块
    matplotlib.rcParams["font.sans-serif"] = [
        "Noto Sans CJK SC", "WenQuanYi Micro Hei", "Source Han Sans SC",
        "Microsoft YaHei", "SimHei", "PingFang SC", "DejaVu Sans",
    ]
    matplotlib.rcParams["axes.unicode_minus"] = False

    ts, zs, targets, vzs, codes = load(args.csv)
    if not ts:
        print("CSV 里没有数据：%s" % args.csv)
        return 2

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9.0, 6.0), sharex=True,
                                   gridspec_kw={"height_ratios": [2.1, 1.0]})
    ax1.plot(ts, zs, color="#1f77b4", lw=1.8, label="飞行高度 z")
    tgt = [v for v in targets if v == v]                # 去掉 nan
    if tgt:
        ax1.axhline(tgt[-1], color="#2ca02c", ls="--", lw=1.2,
                    label="目标高度 %.2f m" % tgt[-1])
    ax2.plot(ts, vzs, color="#7f7f7f", lw=1.4, label="速度指令 vz (m/s)")
    ax2.axhline(0.0, color="#bbbbbb", lw=0.8)
    ax2.set_xlabel("时间 (s)")

    seen = set()
    for i, cs in enumerate(codes):
        for code in cs:
            if code in seen:
                continue
            seen.add(code)
            level = LEVEL_OF.get(code, "warn")
            color = COLOR_OF[level]
            for ax in (ax1, ax2):
                ax.axvline(ts[i], color=color, ls=":", lw=1.1, alpha=0.85)
            ax1.annotate(code, xy=(ts[i], zs[i]), xytext=(4, 8),
                         textcoords="offset points", fontsize=8, color=color,
                         rotation=90, va="bottom")

    ax1.set_ylabel("高度 (m)")
    ax2.set_ylabel("vz 指令 (m/s)")
    ax1.grid(alpha=0.25)
    ax2.grid(alpha=0.25)
    ax1.legend(loc="upper right", fontsize=9)
    ax2.legend(loc="upper right", fontsize=9)
    title = args.title or os.path.basename(args.csv)
    fig.suptitle(title, fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.96))

    out_dir = os.path.dirname(os.path.abspath(args.out))
    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir)
    fig.savefig(args.out, dpi=args.dpi)
    print("saved: %s（%d 个采样点，告警种类 %s）" % (args.out, len(ts), sorted(seen) or "无"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
