# -*- coding: utf-8 -*-
"""命令行入口：不需要 ROS，直接跑示例轨迹的重定时。

用法::

    python3 main.py                 # 示例：6 关节步态周期，输出三张表
    python3 main.py --vmax 3.0      # 改全局速度上限（按关节数等比）
    python3 main.py --json out.json
"""

import argparse
import json
import sys

from . import cli
from .core import Limits, build_trajectory, max_time_scale
from .examples import JOINT_NAMES, example


def _say(text=""):
    """GBK 控制台下安全打印。"""
    try:
        print(text)
    except UnicodeEncodeError:
        enc = getattr(sys.stdout, "encoding", None) or "ascii"
        print(text.encode(enc, "replace").decode(enc))


def main(argv=None):
    ap = argparse.ArgumentParser(description="关节轨迹时间最优重定时")
    ap.add_argument("--dt", type=float, default=0.001, help="采样步长 s")
    ap.add_argument("--vmax", type=float, default=0.0, help="全局速度上限（0=用示例值）")
    ap.add_argument("--amax", type=float, default=0.0, help="全局加速度上限（0=用示例值）")
    ap.add_argument("--json", default="", help="把结果写入 JSON")
    args = ap.parse_args(argv)

    tk, qk, vmax, amax = example()
    if args.vmax > 0:
        vmax = [args.vmax] * len(vmax)
    if args.amax > 0:
        amax = [args.amax] * len(amax)

    traj = build_trajectory(tk, qk, dt=args.dt)
    limits = Limits(vmax, amax)

    _say("## 概况")
    _say(cli.summarize(traj, limits, JOINT_NAMES))
    _say()
    _say("## 重定时后各关节占用率（1.00 = 贴着上限）")
    _say(cli.utilization_table(traj, limits, JOINT_NAMES))
    _say()
    _say("## 抖动与峰值对比")
    _say(cli.jerk_table(traj, limits))
    _say()

    r = max_time_scale(traj, limits)
    _say("结论：α = %.3f，时长 %.3f s → %.3f s（缩短 %.1f%%），受限关节 `%s`（%s）。"
         % (r["alpha"], r["duration_before"], r["duration_after"],
            100 * (1 - r["duration_after"] / r["duration_before"]),
            JOINT_NAMES[r["joint"]],
            "速度" if r["limited_by"] == "speed" else "加速度"))

    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(cli.json_summary(traj, limits, JOINT_NAMES), f,
                      ensure_ascii=False, indent=2)
        _say("已写入 %s" % args.json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
