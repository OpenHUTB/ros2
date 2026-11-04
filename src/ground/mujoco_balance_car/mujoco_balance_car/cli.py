#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""命令行入口（包内实现；包根的 main.py 只做转发）。

退出码约定（仓库 #140/#143 的修复口径：失败必须返回非 0）：

    0  成功（含"没装 matplotlib 跳过绘图"）
    1  运行失败 / 自检未通过
    2  参数错误（argparse 默认）

用法：

    python3 main.py                      # 默认跑完整离线实验并导出 CSV/PNG
    python3 main.py --gui                # 打开可视化窗口（需要 mujoco-python-viewer）
    python3 main.py --selftest           # 跑单元测试
    python3 main.py --launch             # 通过 ros2 launch 启动（需已 colcon build）
    python3 main.py --lean 0.2 --duration 8 --save_dir shots
"""

import argparse
import os
import subprocess
import sys

from mujoco_balance_car.config import BalanceConfig, ScenarioConfig
from mujoco_balance_car.env import (BalanceEnv, MujocoUnavailable,
                                    describe_environment, mujoco_available)

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_XML = os.path.join(PACKAGE_ROOT, "balance_car.xml")


# ----------------------------------------------------------------------------
# 命令行
# ----------------------------------------------------------------------------
def build_parser():
    cfg = BalanceConfig()
    p = argparse.ArgumentParser(
        description="MuJoCo 两轮自平衡小车（双环 PID）——离线指标评测 + 可选可视化")
    p.add_argument("--headless", action="store_true",
                   help="离线批跑（默认行为，无窗口；导出 CSV/PNG）")
    p.add_argument("--gui", action="store_true",
                   help="打开可视化窗口（需要 mujoco-python-viewer + glfw）")
    p.add_argument("--selftest", action="store_true", help="运行单元测试")
    p.add_argument("--launch", action="store_true",
                   help="通过 ros2 launch 启动（等价于 ros2 launch mujoco_balance_car main.launch.py）")
    p.add_argument("--xml", default=DEFAULT_XML, help="MuJoCo 模型 XML 路径")
    p.add_argument("--duration", type=float, default=cfg.duration,
                   help=f"单场景仿真时长（s，默认 {cfg.duration}）")
    p.add_argument("--lean", type=float, default=cfg.initial_lean,
                   help=f"初始倾角（rad，默认 {cfg.initial_lean}）")
    p.add_argument("--target", type=float, default=None,
                   help="位置阶跃目标（m，默认取 ScenarioConfig.step_position）")
    p.add_argument("--save_dir", default="shots",
                   help="CSV/PNG 输出目录（默认 ./shots）")
    p.add_argument("--kp-angle", type=float, default=cfg.kp_angle, help="倾角环 Kp")
    p.add_argument("--kd-angle", type=float, default=cfg.kd_angle, help="倾角环 Kd")
    p.add_argument("--kp-pos", type=float, default=cfg.kp_pos, help="位置环 Kp")
    p.add_argument("--kd-pos", type=float, default=cfg.kd_pos, help="位置环 Kd")
    p.add_argument("--no-plot", action="store_true", help="即使装了 matplotlib 也不画图")
    return p


def config_from_args(args):
    cfg = BalanceConfig(duration=args.duration, initial_lean=args.lean,
                        kp_angle=args.kp_angle, kd_angle=args.kd_angle,
                        kp_pos=args.kp_pos, kd_pos=args.kd_pos)
    sc = ScenarioConfig()
    if args.target is not None:
        sc.step_position = args.target
    return cfg, sc


# ----------------------------------------------------------------------------
# 子命令
# ----------------------------------------------------------------------------
def run_headless(args):
    """离线批跑：三个场景 + 初始倾角扫描。"""
    from mujoco_balance_car import runner

    print("=" * 64)
    print("  MuJoCo 两轮自平衡小车 —— 离线指标评测（无需图形界面）")
    print("=" * 64)
    print(f"环境: {describe_environment()}")
    print(f"模型: {args.xml}")

    cfg, sc = config_from_args(args)
    try:
        summary = runner.run_all(save_dir=args.save_dir, cfg=cfg, sc=sc,
                                 xml_path=args.xml,
                                 verbose=True)
    except MujocoUnavailable as exc:
        print(f"\n[错误] {exc}")
        return 1
    except FileNotFoundError as exc:
        print(f"\n[错误] {exc}")
        return 1

    if args.no_plot:
        summary.pop("pngs", None)

    pngs = summary.get("pngs") or []
    if pngs:
        print("\n已导出曲线图：")
        for p in pngs:
            print("   ", p)
    # 导出目录里的 CSV 一律列出，便于复算
    csvs = sorted(f for f in os.listdir(args.save_dir) if f.endswith(".csv"))
    if csvs:
        print("已导出 CSV：" + "、".join(csvs))

    # 退出码：关键场景必须真的成功，否则返回 1（不允许"跑了但没收敛"当成功）
    bal = summary["results"][0].metrics
    ok = bool(bal.get("upright")) and bal.get("angle_settle") == bal.get("angle_settle")
    if not ok:
        print("\n[失败] 平衡场景未收敛：请检查 PID 增益或初始倾角是否超出可恢复范围。")
        return 1
    print("\n[完成] 平衡场景收敛，指标已导出。")
    return 0


def run_gui(args):
    """可视化窗口：保留原有交互（R 复位、↑/↓ 施加扰动、ESC 退出）。"""
    try:
        import glfw
        import mujoco_viewer
    except Exception as exc:  # noqa: BLE001
        print("[错误] GUI 模式需要 mujoco-python-viewer 与 glfw：pip install mujoco-python-viewer glfw")
        print(f"       原始错误：{exc}")
        print("       无图形界面时请用：python3 main.py --headless")
        return 1

    from mujoco_balance_car.controller import BalanceController
    cfg, _sc = config_from_args(args)
    env = BalanceEnv(xml_path=args.xml, timestep=cfg.control_dt)
    ctrl = BalanceController(cfg, target_position=0.0)

    viewer = mujoco_viewer.MujocoViewer(env.model, env.data)
    viewer.cam.distance = 3.5
    viewer.cam.elevation = -20

    print("=== MuJoCo 两轮自平衡小车 PID 仿真 ===")
    print("R键：重置仿真 | 方向键↑/↓：施加扰动 | ESC：退出")

    env.reset(lean=cfg.initial_lean, x=0.0)
    ctrl.reset()
    try:
        while viewer.is_alive:
            window = viewer.window
            if glfw.get_key(window, glfw.KEY_R) == glfw.PRESS:
                env.reset(lean=cfg.initial_lean, x=0.0)
                ctrl.reset()
                ctrl.set_target(0.0)
            if glfw.get_key(window, glfw.KEY_UP) == glfw.PRESS:
                env.disturb(1.5)
            if glfw.get_key(window, glfw.KEY_DOWN) == glfw.PRESS:
                env.disturb(-1.5)

            torque, _target_lean = ctrl.compute(env.x, env.angle, env.timestep)
            env.step(torque)
            viewer.render()
    finally:
        viewer.close()
    return 0


def run_selftest():
    """跑单元测试（复用 test/test_balance_logic.py）。"""
    test_file = os.path.join(PACKAGE_ROOT, "test", "test_balance_logic.py")
    if not os.path.isfile(test_file):
        print(f"[错误] 找不到测试文件：{test_file}")
        return 1
    print(f"[自检] {test_file}")
    return subprocess.call([sys.executable, test_file])


def run_launch(args):
    """通过 ros2 launch 启动（需要先 colcon build 本功能包）。"""
    cmd = ["ros2", "launch", "mujoco_balance_car", "main.launch.py",
           f"save_dir:={os.path.abspath(args.save_dir)}",
           f"duration:={args.duration}"]
    print("[launch] " + " ".join(cmd))
    try:
        return subprocess.call(cmd)
    except FileNotFoundError:
        print("[错误] 找不到 ros2 命令：请先 source ROS 2 环境，或用 python3 main.py --headless")
        return 1


# ----------------------------------------------------------------------------
def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.selftest:
        return run_selftest()
    if args.launch:
        return run_launch(args)
    if args.gui:
        if not mujoco_available():
            print("[错误] 未安装 mujoco，无法打开可视化。pip install 'mujoco>=3.0.0'")
            return 1
        return run_gui(args)
    return run_headless(args)


if __name__ == "__main__":
    sys.exit(main())
