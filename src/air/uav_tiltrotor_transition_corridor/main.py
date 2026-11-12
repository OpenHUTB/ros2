#!/usr/bin/env python3
"""离线入口：跑场景、打印指标、可选画图。

用法::

    python main.py                      # 跑全部场景并打印指标表
    python main.py --scenario nominal   # 只跑一个
    python main.py --scenario nominal --duration 130 --plots ./fig
"""

from __future__ import annotations

import argparse
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src"))

from uav_tiltrotor_transition_corridor import Corridor  # noqa: E402
from sim import scenarios  # noqa: E402


def print_corridor() -> None:
    print("== 走廊特征 ==")
    print(f"  {'高度':>8}{'失速速度':>10}{'悬停极限':>10}{'最大平飞':>10}{'死区宽度':>10}")
    for alt in (0.0, 2000.0, 4000.0):
        c = Corridor(alt_m=alt)
        dead = c.stall_speed - c.hover_speed_limit
        print(f"  {int(alt):>6} m{c.stall_speed:>10.2f}{c.hover_speed_limit:>10.2f}"
              f"{c.max_level_speed():>10.2f}{dead:>10.2f}")


def run_scenarios(keys: list[str], duration: float | None) -> None:
    print("\n== 场景指标 ==")
    head = f"  {'场景':<16}{'中止':<6}{'能量Wh':>9}{'高度误差m':>11}{'最大下沉m/s':>12}{'升力裕度°':>11}"
    print(head)
    print("  " + "-" * (len(head) - 2))
    for key in keys:
        sc = scenarios.SCENARIOS[key]
        sim = scenarios.build(key)
        res = sim.run(duration or sc.duration)
        s = res.summary()
        flag = "是" if s["aborted"] else "否"
        print(f"  {sc.title:<16}{flag:<8}{s['energy_wh']:>9.2f}{s['alt_error_m']:>11.2f}"
              f"{s['max_sink_mps']:>12.2f}{s['min_lift_margin_deg']:>11.1f}")
        if s["aborted"]:
            print(f"    中止原因：{s['abort_reason']}")


def main() -> None:
    ap = argparse.ArgumentParser(description="倾转旋翼过渡走廊：离线实验入口")
    ap.add_argument("--scenario", default="all", help="场景名或 all")
    ap.add_argument("--duration", type=float, default=None, help="覆盖场景时长，s")
    ap.add_argument("--plots", default=None, help="图片输出目录（走 sim/experiments.py）")
    args = ap.parse_args()

    print_corridor()
    keys = list(scenarios.SCENARIOS) if args.scenario == "all" else [args.scenario]
    for k in keys:
        if k not in scenarios.SCENARIOS:
            raise SystemExit(f"未知场景：{k}（可选：{', '.join(scenarios.SCENARIOS)}）")
    run_scenarios(keys, args.duration)

    if args.plots:
        from sim import experiments

        out = pathlib.Path(args.plots)
        out.mkdir(parents=True, exist_ok=True)
        experiments.plot_corridor(out)
        experiments.plot_profile(out)
        experiments.plot_guard(out)
        experiments.plot_altitude(out)
        print(f"\n图片已写入 {out}")


if __name__ == "__main__":
    main()
