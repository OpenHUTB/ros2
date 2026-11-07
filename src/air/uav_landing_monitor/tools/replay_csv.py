#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把一段真机/仿真的 CSV 回放进当前判据，看看现在会报什么 —— 用于确定性复现与回归。

    python3 tools/replay_csv.py landing.csv            # 打印告警与报告
    python3 tools/replay_csv.py landing.csv --plot out.png

CSV 就是节点 `monitor/csv` 产出的原始序列（`t,phase,z,target_z,vz_cmd,status,alerts`），
回放时只取 `t / z / vz_cmd / status / target_z`，**忽略原文件里的 alerts 列**，
所以同一段数据无论用哪个版本的判据跑，结果都可复现、可比对。
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

from monitor_core import LandingMonitor  # noqa: E402


def replay(path, target_override=None):
    """回放 CSV，返回 (monitor, 统计)。"""
    rows = []
    with open(path, "r", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            try:
                rows.append((float(row["t"]), float(row["z"]), float(row["vz_cmd"]),
                             (row.get("status") or "").strip(),
                             float(row.get("target_z", "nan"))))
            except (KeyError, TypeError, ValueError):
                continue
    if not rows:
        raise SystemExit("CSV 里没有可用数据：%s" % path)

    target = target_override
    if target is None:
        for (_t, _z, _vz, _st, tg) in rows:
            if tg == tg:                                  # 不是 nan
                target = tg
    monitor = LandingMonitor(target_z=target)

    alerts = []
    for (t, z, vz, status, _tg) in rows:
        monitor.on_odom(t, z)
        monitor.on_cmd(t, vz)
        if status:
            monitor.on_status(t, status)
        alerts.extend(monitor.tick(t))
    return monitor, alerts


def main(argv=None):
    ap = argparse.ArgumentParser(description="回放降落过程 CSV，用当前判据重新判定")
    ap.add_argument("csv")
    ap.add_argument("--plot", default="", help="同时画出证据图（需要 matplotlib）")
    ap.add_argument("--title", default="")
    ap.add_argument("--target", type=float, default=None, help="覆盖目标高度")
    args = ap.parse_args(argv)

    monitor, alerts = replay(args.csv, args.target)
    rep = monitor.report()
    print("回放：%s" % args.csv)
    print("结论：%s   阶段：%s   告警统计：%s" % (rep["verdict"], rep["phase"],
                                          rep["alert_counts"] or "无"))
    for a in alerts:
        print("  [%-5s] %-16s %7.2fs  %s" % (a.level, a.code, a.t, a.msg))

    if args.plot:
        import subprocess
        here = os.path.dirname(os.path.abspath(__file__))
        cmd = [sys.executable, os.path.join(here, "plot_report.py"), args.csv, "--out", args.plot]
        if args.title:
            cmd += ["--title", args.title]
        subprocess.check_call(cmd)
    return 0


if __name__ == "__main__":
    sys.exit(main())
