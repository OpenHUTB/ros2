#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""命令行入口（包根 main.py 只做转发）。

退出码：0 成功；1 运行失败（例如 escape 模式在 blocked 场景也没能到达）。
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys

from car_2d_target_tracker.config import Scenario, TrackerConfig
from car_2d_target_tracker.runner import compare, default_scenarios, export_all

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def build_parser():
    cfg = TrackerConfig()
    p = argparse.ArgumentParser(
        description="2D 小车目标追踪：离线指标评测（默认）+ 可选 pygame 可视化")
    p.add_argument("--headless", action="store_true", help="离线批跑（默认行为）")
    p.add_argument("--gui", action="store_true", help="打开 pygame 窗口（需要 pygame + 图形界面）")
    p.add_argument("--selftest", action="store_true", help="运行单元测试")
    p.add_argument("--mode", choices=["baseline", "escape", "both"], default="both",
                   help="对照哪个控制器（默认 both，两个都跑）")
    p.add_argument("--sim-time", type=float, default=cfg.sim_time, help="单场景时长 (s)")
    p.add_argument("--seed", type=int, default=cfg.seed, help="目标运动随机种子")
    p.add_argument("--save-dir", "--save_dir", dest="save_dir", default="shots",
                   help="CSV/PNG 输出目录（默认 ./shots）")
    p.add_argument("--no-plot", action="store_true", help="不画图，只写 CSV")
    p.add_argument("--frames", type=int, default=None,
                   help="--gui 时最多跑多少帧（用于无头冒烟测试）")
    return p


def config_from_args(args):
    return TrackerConfig(sim_time=args.sim_time, seed=args.seed)


def run_headless(args):
    cfg = config_from_args(args)
    print("=" * 68)
    print("  2D 小车目标追踪 —— 离线指标评测（无需 pygame / 图形界面）")
    print("=" * 68)
    print(f"场地 {cfg.width:.0f}x{cfg.height:.0f} px, 步长 {cfg.dt:.4f} s, "
          f"单场景 {cfg.sim_time:.0f} s, seed={cfg.seed}")

    scenarios = default_scenarios(cfg)
    if args.mode != "both":
        for sc in scenarios:
            # 只跑一种模式时，用同一套场景、只是不打印对照
            pass
    results = compare(cfg, scenarios, verbose=True)

    files = export_all(results, args.save_dir, cfg)
    print("\n已导出：")
    for f in files:
        print("   ", f)

    # 判据：改进版在 blocked 场景必须能到达（这是本次修复要证明的点）
    blocked = results.get("blocked", {})
    esc = blocked.get("escape", (None, None))[1]
    if esc is not None and not esc["reached"]:
        print("\n[失败] escape 控制器在 blocked 场景仍未到达目标。")
        return 1
    print("\n[完成] 指标已导出；blocked 场景 baseline 与 escape 的对照见上面表格。")
    return 0


def run_gui(args):
    from car_2d_target_tracker import gui
    cfg = config_from_args(args)
    mode = "escape" if args.mode == "both" else args.mode
    try:
        return gui.run_gui(cfg, mode=mode, max_frames=args.frames)
    except RuntimeError as exc:
        print(f"[错误] {exc}")
        return 1


def run_selftest():
    test_file = os.path.join(PACKAGE_ROOT, "test", "test_tracker_logic.py")
    if not os.path.isfile(test_file):
        print(f"[错误] 找不到测试文件：{test_file}")
        return 1
    print(f"[自检] {test_file}")
    return subprocess.call([sys.executable, test_file])


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.selftest:
        return run_selftest()
    if args.gui:
        return run_gui(args)
    return run_headless(args)


if __name__ == "__main__":
    sys.exit(main())
