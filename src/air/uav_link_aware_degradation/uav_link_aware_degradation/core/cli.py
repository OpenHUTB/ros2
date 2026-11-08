#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_link_aware_degradation 统一入口。

    # 跑完整实验矩阵（4 档链路 × 3 种策略 × 3 个种子）
    python3 main.py run --out results

    # 只跑一个场景，生成对比动图与指标图
    python3 main.py demo --out results --profile outage

    # 纯逻辑自检（不需要 MuJoCo）
    python3 main.py selftest
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from uav_link_aware_degradation.core.experiment import run_matrix   # noqa: E402
from uav_link_aware_degradation.core.link import default_profiles   # noqa: E402
from uav_link_aware_degradation.core.render import (                 # noqa: E402
    build_comparison_gif, build_metrics_figure)
from uav_link_aware_degradation.core.supervisor import SupervisorParams  # noqa: E402


def _cmd_run(args) -> int:
    payload = run_matrix(Path(args.out), profiles=args.profiles or None,
                         seeds=tuple(range(args.seeds)), duration=args.duration)
    print(json.dumps(payload["aggregate"], ensure_ascii=False, indent=2))
    return 0


def _cmd_demo(args) -> int:
    outdir = Path(args.out)
    payload = run_matrix(outdir, profiles=[args.profile], seeds=(0,),
                         duration=args.duration)
    results = {"aggregate": payload["aggregate"], "outdir": str(outdir)}
    if args.gif:
        build_comparison_gif(
            args.profile,
            outdir / f"trace_{args.profile}_hold_last_s0.npz",
            outdir / f"trace_{args.profile}_supervised_s0.npz",
            "沿用最后指令", "分级降级",
            Path(args.gif), fps=args.fps)
        print(f"动图：{args.gif}")
    if args.figure:
        build_metrics_figure(results, args.profile, Path(args.figure))
        print(f"指标图：{args.figure}")
    return 0


def _cmd_selftest(_args) -> int:
    from uav_link_aware_degradation.core.link import LinkEmulator, LinkMonitor
    from uav_link_aware_degradation.core.mission import Mission
    from uav_link_aware_degradation.core.supervisor import DegradationSupervisor
    import numpy as np

    prof = default_profiles()["outage"]
    emu = LinkEmulator(prof, seed=0)
    ds = [emu.transmit(t) for t in np.arange(0, 20, 0.05)]
    lost = sum(not d.delivered for d in ds)
    print(f"outage 档 20 s 内丢包/未送达 {lost}/{len(ds)}")

    mon = LinkMonitor(20.0)
    for d in ds[:200]:
        if d.delivered:
            mon.push(d.send_t, d.arrival)
    sv = DegradationSupervisor(SupervisorParams())
    mission = Mission(home=np.array([0.0, 0.0, 1.2]),
                      waypoints=[np.array([5.0, 0.0, 1.5])])
    for t in np.arange(2.0, 6.0, 0.05):
        st = mon.update(t)
        sv.update(t, st, np.array([0.0, 0.0, 1.2]), mission)
    print(f"末态模式 {sv.mode.value}，切换 {sv.switches} 次")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="通信链路分级降级实验")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_run = sub.add_parser("run", help="跑实验矩阵")
    p_run.add_argument("--out", default="results")
    p_run.add_argument("--profiles", nargs="*", default=None)
    p_run.add_argument("--seeds", type=int, default=3)
    p_run.add_argument("--duration", type=float, default=45.0)
    p_run.set_defaults(func=_cmd_run)

    p_demo = sub.add_parser("demo", help="单场景 + 动图 + 指标图")
    p_demo.add_argument("--out", default="results")
    p_demo.add_argument("--profile", default="outage")
    p_demo.add_argument("--gif", default=None)
    p_demo.add_argument("--figure", default=None)
    p_demo.add_argument("--fps", type=float, default=12.5)
    p_demo.add_argument("--duration", type=float, default=45.0)
    p_demo.set_defaults(func=_cmd_demo)

    p_self = sub.add_parser("selftest", help="纯逻辑自检")
    p_self.set_defaults(func=_cmd_selftest)

    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
