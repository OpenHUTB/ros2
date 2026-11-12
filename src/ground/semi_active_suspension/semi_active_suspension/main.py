"""命令行入口：跑四个工况，打印指标表，可选地生成图片。

    python3 -m semi_active_suspension.main --output results

只依赖标准库，出图部分用的是包内自带的 PNG 编码。
"""
from __future__ import annotations

import argparse
import os

from . import render, scenarios, scene, sim

ROWS = [
    ("accel_rms", "车身加速度 RMS", "m/s^2", 1, "{:6.3f}"),
    ("accel_peak", "车身加速度峰值", "m/s^2", 1, "{:6.2f}"),
    ("pitch_deg", "俯仰角峰值", "deg", 1, "{:6.2f}"),
    ("roll_deg", "侧倾角峰值", "deg", 1, "{:6.2f}"),
    ("load_min", "最小轮荷", "N", 1, "{:7.0f}"),
]


def run_all(damper=None):
    pairs = []
    for scenario in scenarios.all_scenarios():
        pairs.append(sim.run_pair(scenario, damper=damper))
    return pairs


def print_table(pairs) -> None:
    header = "%-14s %-22s %-8s %12s %12s %8s" % (
        "工况", "指标", "单位", "被动", "半主动", "变化")
    print(header)
    print("-" * len(header))
    for passive, active in pairs:
        for key, label, unit, _scale, fmt in ROWS:
            before = passive.metrics[key]
            after = active.metrics[key]
            if abs(before) < 1e-9:
                continue
            change = (after - before) / abs(before) * 100.0
            print("%-14s %-22s %-8s %12s %12s %7.1f%%" % (
                passive.scenario.name, label, unit,
                fmt.format(before).strip(), fmt.format(after).strip(), change))
        print()


def write_images(pairs, output: str) -> list:
    os.makedirs(output, exist_ok=True)
    jobs = [
        ("susp_bump.png", lambda: scene.bump_figure(*pairs[0])),
        ("susp_dive.png", lambda: scene.dive_figure(*pairs[1])),
        ("susp_roll.png", lambda: scene.roll_figure(*pairs[2])),
        ("susp_summary.png", lambda: scene.summary_figure(pairs)),
    ]
    written = []
    for name, build in jobs:
        path = os.path.join(output, name)
        render.write_png(path, build())
        written.append(path)
    return written


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="半主动悬架车身姿态控制仿真")
    parser.add_argument("--output", default="", help="图片输出目录，留空则不出图")
    parser.add_argument("--scenario", default="", help="只跑指定工况")
    parser.add_argument("--no-images", action="store_true", help="只打印指标")
    parser.add_argument("--offline", action="store_true", help="兼容开关，无网络依赖")
    args = parser.parse_args(argv)

    if args.scenario:
        scenario = scenarios.by_name(args.scenario)
        pairs = [sim.run_pair(scenario)]
    else:
        pairs = run_all()

    print_table(pairs)

    if args.output and not args.no_images:
        for path in write_images(pairs, args.output):
            print("已写入 %s" % path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
