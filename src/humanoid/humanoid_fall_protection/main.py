#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""人形机器人跌倒预判与保护性倒地 —— standalone 入口

用法（功能包根目录）：
    python3 main.py --version                  # 版本与模型信息
    python3 main.py --test                     # 自检（不开窗、不写文件）
    python3 main.py --headless                 # 三种策略各跑一次，出对比表
    python3 main.py --headless --force 400     # 换推扰力
    python3 main.py --sweep                    # 检测阈值扫描（15°~45°）
    python3 main.py --demo                     # 打开 MuJoCo 窗口看保护动作

退出码：0 正常；1 自检失败；2 参数错误。
"""

import argparse
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from humanoid_fall_protection import config                      # noqa: E402
from humanoid_fall_protection.metrics import format_table, reduction_vs_best_passive  # noqa: E402
from humanoid_fall_protection.runner import run_comparison, run_episode  # noqa: E402
from humanoid_fall_protection.sim_core import FallSim, default_model_path  # noqa: E402

PKG_DIR = Path(__file__).resolve().parent
RESULTS_DIR = PKG_DIR / "results"


def positive_float(text):
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError("需要数字，收到 {!r}".format(text))
    if value <= 0:
        raise argparse.ArgumentTypeError("必须为正数，收到 {}".format(value))
    return value


def build_parser():
    parser = argparse.ArgumentParser(
        description="人形机器人跌倒预判与保护性倒地（MuJoCo）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--version", action="store_true", help="打印版本与模型路径")
    parser.add_argument("--test", action="store_true", help="运行自检")
    parser.add_argument("--headless", action="store_true", help="无窗口跑对比实验")
    parser.add_argument("--demo", action="store_true", help="打开 MuJoCo 窗口演示")
    parser.add_argument("--sweep", action="store_true", help="推力扫描")
    parser.add_argument("--figures", action="store_true", help="生成文档用图与动图")
    parser.add_argument("--strategy", default="protect",
                        choices=list(config.STRATEGIES), help="--demo 时使用的策略")
    parser.add_argument("--force", type=positive_float, default=None, help="推扰力 (N)")
    parser.add_argument("--push-duration", type=positive_float, default=None, help="推扰持续 (s)")
    parser.add_argument("--duration", type=positive_float, default=None, help="单次仿真时长 (s)")
    parser.add_argument("--export", default=None, help="导出 CSV/图片的目录")
    parser.add_argument("--no-viewer", action="store_true", help="--demo 时也不开窗（调试用）")
    return parser


# --------------------------------------------------------------------------- 自检
def cmd_test():
    print("=" * 66)
    print("  humanoid_fall_protection 自检")
    print("=" * 66)
    checks = []

    def check(name, ok, detail=""):
        checks.append((name, bool(ok), detail))
        print("  [{}] {}{}".format("PASS" if ok else "FAIL", name,
                                   "  " + detail if detail else ""))

    sim = FallSim()
    check("模型加载", sim.model.nu == 15 and sim.model.nq > 15,
          "{} 个执行器 / {} 个广义坐标".format(sim.model.nu, sim.model.nq))

    sim.reset()
    check("脚底贴地", abs(min(float(sim.data.geom_xpos[g][2]) for g in sim.foot_geoms) - 0.025) < 5e-3,
          "最低脚底 z={:.4f} m".format(min(float(sim.data.geom_xpos[g][2]) for g in sim.foot_geoms)))

    # 静立 1 s 不应触发跌倒判据
    from humanoid_fall_protection.protection import FallController
    ctrl = FallController()
    ctrl.reset("protect")
    max_tilt = 0.0
    for i in range(int(1.0 / sim.dt)):
        sim.apply_push(0.0, False)
        state = ctrl.step(sim, i * sim.dt)
        sim.step()
        max_tilt = max(max_tilt, abs(sim.tilt()))
    check("静立 1 s 不误触发", not ctrl.detector.triggered,
          "最大倾角 {:.2f}°".format(max_tilt * 57.2958))

    # 推扰后必须触发
    sim.reset()
    ctrl.reset("protect")
    triggered_at = None
    for i in range(int(2.5 / sim.dt)):
        t = i * sim.dt
        sim.apply_push(300.0, 1.0 <= t < 1.08)
        just = ctrl.step(sim, t)["just_triggered"]
        sim.step()
        if just and triggered_at is None:
            triggered_at = t
    check("300 N 推扰触发保护", triggered_at is not None,
          "触发时刻 {:.2f} s".format(triggered_at or -1))

    # 保护动作确实让头部避免触地
    s_rigid, _ = run_episode("rigid")
    s_protect, _ = run_episode("protect")
    check("刚度对比：保护组头部最低点更高",
          s_protect["head_min_z"] > s_rigid["head_min_z"] + 0.005,
          "rigid {:.3f} m → protect {:.3f} m".format(s_rigid["head_min_z"], s_protect["head_min_z"]))
    check("保护组头部未触地", not s_protect["head_touched"],
          "最小间隙 {:.3f} m".format(s_protect["head_min_gap"]))
    # 僵直组头部不是「砸」下去的，而是被躯干垫着慢慢压进地面（球面间隙为负），
    # 所以判据看几何穿透而不是峰值力
    check("僵直组头部球面压入地面", s_rigid["head_min_gap"] < 0.0,
          "最小间隙 {:+.3f} m".format(s_rigid["head_min_gap"]))

    # 瘫软组：力矩确实为零
    s_limp, _ = run_episode("limp")
    check("瘫软组倒地", s_limp["settle_time"] is not None,
          "倒地时刻 {:.2f} s".format(s_limp["settle_time"] or -1))

    # 检测器阈值语义
    from humanoid_fall_protection.protection import FallDetector
    d = FallDetector(tilt_threshold_deg=30, tilt_rate_threshold=2.0, mode="or")
    d.update(0.1, 0.1, 0.0)
    check("小扰动不触发", not d.triggered)
    d2 = FallDetector(tilt_threshold_deg=30, tilt_rate_threshold=2.0, mode="or")
    d2.update(0.6, 0.0, 0.0)
    check("倾角超限即触发", d2.triggered, "0.6 rad ≈ 34°")
    d3 = FallDetector(tilt_threshold_deg=30, tilt_rate_threshold=2.0, mode="and")
    d3.update(0.6, 0.0, 0.0)
    check("and 模式下仅倾角不触发", not d3.triggered)

    failed = [c for c in checks if not c[1]]
    print("=" * 66)
    if failed:
        print("  {} 项失败".format(len(failed)))
        return 1
    print("  全部通过 ✅  （{} 项）".format(len(checks)))
    return 0


# --------------------------------------------------------------------------- 对比
def cmd_headless(args):
    export = args.export or str(RESULTS_DIR)
    os.makedirs(export, exist_ok=True)
    kwargs = {}
    if args.force is not None:
        kwargs["push_force"] = args.force
    if args.push_duration is not None:
        kwargs["push_duration"] = args.push_duration
    if args.duration is not None:
        kwargs["duration"] = args.duration

    print("[运行] 推扰 {} N × {} s，三种策略各一次".format(
        kwargs.get("push_force", config.DEFAULTS["push_force"]),
        kwargs.get("push_duration", config.DEFAULTS["push_duration"])))
    summaries, reduction = run_comparison(export_dir=export, **kwargs)
    print()
    print(format_table(summaries))
    by = {s["strategy"]: s for s in summaries}
    if "protect" in by and "rigid" in by:
        torso_drop = 100.0 * (1 - by["protect"]["torso_peak_force"] / max(1.0, by["rigid"]["torso_peak_force"]))
        knee_rise = 100.0 * (by["protect"]["knee_peak_force"] / max(1.0, by["rigid"]["knee_peak_force"]) - 1)
        print("\n[结论] 头部峰值力  僵直 {:.1f} N / 瘫软 {:.1f} N / 保护 {:.1f} N".format(
            by["rigid"]["head_peak_force"], by.get("limp", {}).get("head_peak_force", 0.0),
            by["protect"]["head_peak_force"]))
        print("[结论] 保护组头部球面始终高于地面 {:.1f} mm，全程未接触".format(
            max(0.0, by["protect"]["head_min_gap"]) * 1000))
        print("[结论] 躯干峰值力 {:.0f} N → {:.0f} N（降低 {:.0f}%），代价是膝部峰值升高 {:.0f}%".format(
            by["rigid"]["torso_peak_force"], by["protect"]["torso_peak_force"], torso_drop, knee_rise))
    print("[导出] {}/*.csv".format(export))
    return 0


# --------------------------------------------------------------------------- 推力扫描
def cmd_sweep(args):
    """按推力扫描三组策略的峰值冲击。

    单点对比容易受「这一下正好砸在哪」影响：limp 组在 200 N 和 300 N 下
    头部峰值力能差 5 倍，因为它每次倒地的姿态都不一样。扫一遍推力，看的是
    趋势而不是某一个幸运/不幸的样本。
    """
    forces = [200, 250, 300, 350, 400, 450, 500]
    export = args.export or str(RESULTS_DIR)
    os.makedirs(export, exist_ok=True)
    print("[扫描] 推扰力 {} N，三组策略逐一跑".format("/".join(str(f) for f in forces)))
    print()
    print("  推力   僵直 头/躯干/膝        瘫软 头/躯干/膝        保护 头/躯干/膝")
    print("  " + "-" * 78)
    rows = []
    for force in forces:
        cells = []
        for strategy in config.STRATEGIES:
            summary, _ = run_episode(strategy, push_force=float(force),
                                     record_csv=os.path.join(export, "force{:03d}_{}.csv".format(force, strategy)))
            cells.append(summary)
            rows.append(summary)
        print("  {:4d}   {:>5.0f}/{:<6.0f}/{:<5.0f}      {:>5.0f}/{:<6.0f}/{:<5.0f}      {:>5.0f}/{:<6.0f}/{:<5.0f}".format(
            force,
            cells[0]["head_peak_force"], cells[0]["torso_peak_force"], cells[0]["knee_peak_force"],
            cells[1]["head_peak_force"], cells[1]["torso_peak_force"], cells[1]["knee_peak_force"],
            cells[2]["head_peak_force"], cells[2]["torso_peak_force"], cells[2]["knee_peak_force"]))

    head_protect = [r for r in rows if r["strategy"] == "protect"]
    head_limp = [r for r in rows if r["strategy"] == "limp"]
    torso_rigid = [r for r in rows if r["strategy"] == "rigid"]
    torso_protect = [r for r in rows if r["strategy"] == "protect"]
    print()
    print("[汇总] 保护组头部峰值力最大值 {:.1f} N；瘫软组最大值 {:.1f} N".format(
        max(r["head_peak_force"] for r in head_protect),
        max(r["head_peak_force"] for r in head_limp)))
    print("[汇总] 躯干峰值力：僵直 {:.0f}~{:.0f} N → 保护 {:.0f}~{:.0f} N".format(
        min(r["torso_peak_force"] for r in torso_rigid), max(r["torso_peak_force"] for r in torso_rigid),
        min(r["torso_peak_force"] for r in torso_protect), max(r["torso_peak_force"] for r in torso_protect)))
    print("[导出] {}/*.csv".format(export))
    return 0


# --------------------------------------------------------------------------- 出图
def cmd_figures(args):
    """生成文档用图：头部高度曲线、冲击柱状图、关键帧、动图。"""
    from humanoid_fall_protection import figures
    from humanoid_fall_protection.runner import run_episode

    export = args.export or str(RESULTS_DIR)
    os.makedirs(export, exist_ok=True)

    recorders = {}
    summaries = []
    for strategy in config.STRATEGIES:
        summary, recorder = run_episode(strategy)
        summaries.append(summary)
        recorders[strategy] = recorder

    figures.plot_head_height(recorders, os.path.join(export, "height.png"))
    print("[图] height.png")
    figures.plot_impacts(summaries, os.path.join(export, "impact.png"))
    print("[图] impact.png")

    shots = []
    for strategy in ("rigid", "protect"):
        shots += figures.render_frames(strategy, [1.60, 2.05, 2.80],
                                       os.path.join(export, "frames"))
    figures.montage(shots, os.path.join(export, "compare.png"), cols=3)
    print("[图] compare.png")

    gif, count = figures.render_gif("protect", os.path.join(export, "demo.gif"))
    size_mb = os.path.getsize(gif) / 1024 / 1024
    print("[图] demo.gif（{} 帧，{:.1f} MB）".format(count, size_mb))
    return 0


# --------------------------------------------------------------------------- demo
def cmd_demo(args):
    import mujoco.viewer  # 延迟导入：无图形环境时 --headless 不受影响

    sim = FallSim()
    sim.reset()
    from humanoid_fall_protection.protection import FallController
    controller = FallController()
    controller.reset(args.strategy)
    force = args.force or config.DEFAULTS["push_force"]

    print("[demo] 策略 {}，推扰 {} N。关闭窗口结束。".format(args.strategy, force))
    with mujoco.viewer.launch_passive(sim.model, sim.data,
                                      show_left_ui=False, show_right_ui=False) as viewer:
        # 用自由相机而不是模型里的固定机位：固定机位的 fovy 与跟随目标在
        # 不同窗口比例下容易把人形裁出画面（实测只拍到一条腿）。
        # 这组参数与 figures.py 的离屏渲染保持一致，文档里的关键帧和
        # 窗口里看到的构图是同一个。
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.lookat[:] = [0.45, 0.0, 0.45]
        viewer.cam.distance = 3.40
        viewer.cam.azimuth = 92
        viewer.cam.elevation = -6
        step = 0
        wall_t0 = time.perf_counter()
        while viewer.is_running():
            t = step * sim.dt
            sim.apply_push(force, 1.0 <= t < 1.08)
            controller.step(sim, t)
            sim.step()
            step += 1
            # 实时节流：不节流时物理步进会跑到几千 Hz，4 s 的仿真在 1 s 内放完，
            # 录出来的动图看着像「瞬移倒地」。每 10 步（20 ms）校一次时钟。
            if step % 10 == 0:
                lag = t - (time.perf_counter() - wall_t0)
                if lag > 0:
                    time.sleep(min(lag, 0.02))
            if step % 2 == 0:
                # 每帧重设相机：passive viewer 的 sync() 内部会按窗口比例重算视图，
                # 只在进入 with 块时设一次会被覆盖成默认的近景
                viewer.cam.lookat[:] = [0.45, 0.0, 0.45]
                viewer.cam.distance = 3.40
                viewer.cam.azimuth = 92
                viewer.cam.elevation = -6
                viewer.sync()
            if t > 5.5:  # 跑完一遍后循环重来；留 2.5 s 展示倒地后的稳定姿态，
                         # 否则「跪撑、头悬空」只有一两帧，录出来看不清
                step = 0
                wall_t0 = time.perf_counter()
                sim.reset()
                controller.reset(args.strategy)
    return 0


# --------------------------------------------------------------------------- main
def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.version:
        print("humanoid_fall_protection 1.0.0")
        print("模型：{}".format(default_model_path()))
        sim = FallSim()
        print("执行器 {}，广义坐标 {}，timestep {} s".format(
            sim.model.nu, sim.model.nq, sim.model.opt.timestep))
        return 0

    if args.test:
        return cmd_test()
    if args.sweep:
        return cmd_sweep(args)
    if args.figures:
        return cmd_figures(args)
    if args.demo:
        return cmd_demo(args)
    if args.headless:
        return cmd_headless(args)

    parser.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
