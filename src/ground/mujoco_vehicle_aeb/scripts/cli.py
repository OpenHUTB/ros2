#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""命令行入口（包根的 main.py 只做转发）。

退出码：0 成功；1 运行失败或基线场景没刹住；2 参数错误（argparse 默认）。

用法：

    python3 main.py                     # 默认离线批跑，导出 CSV/PNG
    python3 main.py --gui               # 打开 MuJoCo 窗口（需要图形界面）
    python3 main.py --selftest          # 跑单元测试
    python3 main.py --v-ego 12 --ttc 6  # 改工况与阈值
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys

from aeb_controller import AEBController
from aeb_env import (DEFAULT_MAX_FORCE, MujocoUnavailable, describe_environment,
                     mujoco_available)
from runner import (AEBConfig, run_scenario, sweep, write_metrics_csv,
                    write_track_csv)

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_XML = os.path.join(PACKAGE_ROOT, "models", "aeb_car.xml")


def build_parser():
    cfg = AEBConfig()
    p = argparse.ArgumentParser(
        description="MuJoCo 自动紧急制动（AEB / TTC）——离线指标评测 + 可选可视化")
    p.add_argument("--headless", action="store_true", help="离线批跑（默认行为）")
    p.add_argument("--gui", action="store_true", help="打开 MuJoCo 窗口（需要图形界面）")
    p.add_argument("--selftest", action="store_true", help="运行单元测试")
    p.add_argument("--xml", default=DEFAULT_XML, help="MuJoCo 模型 XML 路径")
    p.add_argument("--ttc", type=float, default=cfg.ttc_threshold, help="TTC 阈值 (s)")
    p.add_argument("--gain", type=float, default=cfg.gain, help="比例增益")
    p.add_argument("--v-ego", type=float, default=cfg.ego_v, help="自车初速度 (m/s)")
    p.add_argument("--v-lead", type=float, default=cfg.lead_v, help="前车初速度 (m/s)")
    p.add_argument("--lead-x", type=float, default=cfg.lead_x, help="前车初始位置 (m)")
    p.add_argument("--sim-time", type=float, default=cfg.sim_time, help="仿真时长 (s)")
    p.add_argument("--max-force", type=float, default=DEFAULT_MAX_FORCE,
                   help="制动力上限 (N)")
    p.add_argument("--save-dir", "--save_dir", dest="save_dir", default="shots",
                   help="CSV/PNG 输出目录（默认 ./shots）")
    p.add_argument("--no-latch", action="store_true",
                   help="关闭制动保持，复现“纯比例 TTC 律”（会出现贴脸才刹的现象）")
    p.add_argument("--latch-floor", type=float, default=cfg.latch_floor,
                   help="制动保持下限 (0~1)")
    p.add_argument("--no-sweep", action="store_true", help="跳过工况扫描（快一些）")
    p.add_argument("--no-plot", action="store_true", help="不画图，只写 CSV")
    return p


def config_from_args(args):
    return AEBConfig(ttc_threshold=args.ttc, gain=args.gain, ego_v=args.v_ego,
                     lead_v=args.v_lead, lead_x=args.lead_x,
                     sim_time=args.sim_time, max_force=args.max_force,
                     latch=not args.no_latch, latch_floor=args.latch_floor)


def _show(name, m):
    def fmt(v, nd=3):
        if isinstance(v, float):
            return "nan" if v != v else f"{v:.{nd}f}"
        return str(v)
    print(f"  [{name:<11}] 最小间距 {fmt(m['min_gap']):>8} m  碰撞 {str(m['collided']):>5}"
          f"  撞时速度 {fmt(m['impact_speed']):>6} m/s  最小TTC {fmt(m['ttc_min']):>6} s"
          f"  制动介入 {fmt(m['brake_onset']):>6} s  制动距离 {fmt(m['stop_distance']):>7} m")


def run_headless(args):
    print("=" * 68)
    print("  MuJoCo 自动紧急制动（AEB / TTC）—— 离线指标评测（无需图形界面）")
    print("=" * 68)
    print(f"环境: {describe_environment()}")
    print(f"模型: {args.xml}")

    cfg = config_from_args(args)
    print(f"工况: v_ego={cfg.ego_v:g} m/s, v_lead={cfg.lead_v:g} m/s, "
          f"lead_x={cfg.lead_x:g} m, TTC阈值={cfg.ttc_threshold:g} s, gain={cfg.gain:g}")

    try:
        tracks, metrics = {}, {}
        tracks["baseline"], metrics["baseline"] = run_scenario(cfg, "baseline", args.xml)
        tracks["no_aeb"], metrics["no_aeb"] = run_scenario(
            AEBConfig(**{**cfg.__dict__, "gain": 0.0}), "no_aeb", args.xml)
        tracks["high_speed"], metrics["high_speed"] = run_scenario(
            AEBConfig(**{**cfg.__dict__, "ego_v": 12.0}), "high_speed", args.xml)

        sweep_rows = []
        if not args.no_sweep:
            sweep_rows = sweep(cfg, xml_path=args.xml)
    except MujocoUnavailable as exc:
        print(f"\n[错误] {exc}")
        return 1
    except FileNotFoundError as exc:
        print(f"\n[错误] {exc}")
        return 1

    print("\n=== 场景指标 ===")
    for name in ("baseline", "no_aeb", "high_speed"):
        _show(name, metrics[name])

    if sweep_rows:
        print("\n=== 工况扫描（初速度 × 增益）===")
        gains = sorted({r["gain"] for r in sweep_rows})
        speeds = sorted({r["ego_v"] for r in sweep_rows})
        head = "  v\\gain " + "".join(f"{g:>10g}" for g in gains)
        print(head)
        for v in speeds:
            cells = []
            for g in gains:
                row = next(r for r in sweep_rows if r["ego_v"] == v and r["gain"] == g)
                mark = "撞" if row["collided"] else " "
                cells.append(f"{row['min_gap']:>8.2f}{mark}")
            print(f"  {v:>6g} " + "".join(cells))
        print("  （数字为最小净间距 m，带『撞』= 发生碰撞）")

    os.makedirs(args.save_dir, exist_ok=True)
    for name, track in tracks.items():
        write_track_csv(track, args.save_dir, name)
    metrics_csv = write_metrics_csv(metrics, sweep_rows, args.save_dir)
    print(f"\n指标 CSV：{metrics_csv}")

    pngs = []
    if not args.no_plot:
        from plotting import export_all
        pngs = export_all(tracks, cfg.ttc_threshold, sweep_rows, args.save_dir)
        if pngs:
            print("已导出曲线图：")
            for p in pngs:
                print("   ", p)

    # 基线必须刹住，否则视为失败（不允许"跑了但没刹住"当成功）
    if metrics["baseline"]["collided"]:
        print("\n[失败] 基线工况发生碰撞：请检查 TTC 阈值 / 增益 / 初速度是否超出能力。")
        return 1
    print("\n[完成] 基线工况未碰撞，指标已导出。")
    return 0


def run_gui(args):
    """MuJoCo 窗口：R 复位、ESC 退出；行为与原版一致（保留实时慢放）。"""
    if not mujoco_available():
        print("[错误] 未安装 mujoco，无法打开可视化。")
        return 1
    import time

    import mujoco.viewer

    from aeb_env import AEBEnv

    cfg = config_from_args(args)
    env = AEBEnv(xml_path=args.xml)
    env.reset(ego_x=0.0, ego_v=cfg.ego_v, lead_x=cfg.lead_x, lead_v=cfg.lead_v)
    ctrl = AEBController(cfg.ttc_threshold, cfg.gain, cfg.v_stop)

    print("=== MuJoCo AEB 仿真（GUI）===")
    print(f"v_ego={cfg.ego_v:g} m/s, TTC 阈值={cfg.ttc_threshold:g} s, gain={cfg.gain:g}")
    print("R键：复位 | ESC：退出")

    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        viewer.cam.lookat[:] = [20, 0, 0]
        viewer.cam.distance = 55
        viewer.cam.elevation = -35
        viewer.cam.azimuth = 90
        wall = time.time()
        while viewer.is_running():
            now = env.time
            desired = now * 1.0                      # 实时慢放
            if desired > time.time() - wall:
                time.sleep(desired - (time.time() - wall))
            ttc = ctrl.compute_ttc(env.gap, env.ego_v, env.lead_v, cfg.v_stop)
            brake = ctrl.get_brake(ttc, env.ego_v)
            env.apply_brake(brake, cfg.max_force)
            env.step()
            viewer.sync()
            if env.time > cfg.sim_time:
                break
    return 0


def run_selftest():
    test_file = os.path.join(PACKAGE_ROOT, "test", "test_aeb_logic.py")
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
