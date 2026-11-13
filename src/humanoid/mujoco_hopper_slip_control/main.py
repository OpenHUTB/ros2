#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 单足弹跳机器人入口。

    python3 main.py --version         版本
    python3 main.py --test            离线自检（不依赖 ROS）
    python3 main.py --headless        跑三项实验，输出表格 + CSV + 图
    python3 main.py --gif             生成演示动图
    python3 main.py --gui             打开交互窗口（需图形环境）
"""

import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mujoco_hopper_slip_control.config as config          # noqa: E402


def _real(p: str) -> Path:
    q = Path(p)
    return q if q.is_absolute() else (HERE / q)


def cmd_version() -> int:
    print(f"{config.PACKAGE_NAME} {config.VERSION}")
    print(f"  mass {config.TOTAL_MASS:.2f} kg, leg k {config.LEG_STIFFNESS:.0f} N/m, "
          f"control {config.CONTROL_HZ:.0f} Hz")
    try:
        import mujoco
        print(f"  mujoco {mujoco.__version__}")
    except ImportError:
        print("  mujoco: 未安装")
    return 0


def cmd_test() -> int:
    """离线自检：控制律数学、模型一致性、一次真实弹跳。"""
    from mujoco_hopper_slip_control.controller import (
        HopperController, Phase, desired_liftoff_speed, foot_placement_angle)

    checks, fails = [], 0

    def check(name, cond, detail=""):
        nonlocal fails
        ok = bool(cond)
        fails += 0 if ok else 1
        checks.append((ok, name, detail))

    # 1 质量换算
    check("总质量 = 各部件之和", abs(config.TOTAL_MASS - 6.0) < 1e-9,
          f"{config.TOTAL_MASS:.2f} kg")

    # 2 静止高度关系
    check("Z_REST = 标称腿长 + 足半径",
          abs(config.Z_REST - (config.LEG_NOMINAL + config.FOOT_RADIUS)) < 1e-9,
          f"{config.Z_REST:.3f} m")

    # 3 起飞速度反解
    v = desired_liftoff_speed(0.55)
    rise = v ** 2 / (2 * config.GRAVITY)
    check("顶点 0.55 m 的起飞速度反解一致", abs(rise - (0.55 - (config.Z_REST - 0.02))) < 1e-6,
          f"vz_des={v:.3f} m/s")

    # 4 落脚角符号：脚在身后（θ>0）应对应 vx < vx_des
    th_slow = foot_placement_angle(vx=0.0, vx_des=0.6)
    check("速度偏慢时落脚角为正（脚放身后）", th_slow > 0, f"θ={th_slow:+.3f} rad")

    # 5 落脚角限幅
    th_ext = foot_placement_angle(vx=-5.0, vx_des=5.0)
    check("落脚角受限幅约束", abs(th_ext) <= config.RAIBERT_ANGLE_LIMIT + 1e-9,
          f"θ={th_ext:+.3f} rad")

    # 6 去抖：单步接触不应立刻切相位
    c = HopperController(debounce=3)
    for _ in range(2):
        c.update(0.30, 0.0, 0.0, 0.05, 0.0, 0.0, contact=True)
    check("单步接触不切相位（去抖生效）", c.phase is Phase.FLIGHT, str(c.phase))

    # 7 去抖：连续 3 步接触应切入支撑相
    c.update(0.30, 0.0, 0.0, 0.05, 0.0, 0.0, contact=True)
    check("连续接触 3 步切入支撑相", c.phase is Phase.STANCE, str(c.phase))

    # 8 注能力只增不减的边界（负增益被截断）
    c2 = HopperController(energy_gain=-5.0)
    check("能量增益下限被截断", c2.energy_gain >= config.ENERGY_GAIN_MIN,
          f"{c2.energy_gain:.2f}")

    # 9 模型能加载且质量与常量一致
    try:
        from mujoco_hopper_slip_control.sim import HopperSim, find_model
        sim = HopperSim()
        check("模型质量与 TOTAL_MASS 一致", abs(sim.mass - config.TOTAL_MASS) < 1e-6,
              f"model {sim.mass:.2f} kg, path {find_model().name}")

        # 10 真实弹跳：跑 8 s 应至少跳起 MIN_HOPS 次且不摔
        from mujoco_hopper_slip_control import metrics
        log = sim.run(8.0)
        sim.close()
        hops = metrics.extract_hops(log)
        st = metrics.steady_stats(hops)
        check("8 s 内弹跳次数达标", st["count"] >= config.MIN_HOPS, f"{st['count']} hops")
        check("稳态顶点接近目标",
              abs(st.get("apex_mean", 0) - config.TARGET_HEIGHT) < 0.10,
              f"{st.get('apex_mean', float('nan')):.3f} m vs {config.TARGET_HEIGHT:.2f}")
        check("全程未摔倒", log["z"].min() > config.FALL_Z, f"z_min={log['z'].min():.3f}")
    except Exception as exc:                       # noqa: BLE001
        check("MuJoCo 仿真可运行", False, f"{type(exc).__name__}: {exc}")

    width = max(len(n) for _, n, _ in checks) + 2
    for ok, name, detail in checks:
        mark = "PASS" if ok else "FAIL"
        print(f"[{mark}] {name:<{width}} {detail}")
    print(f"\n{len(checks) - fails}/{len(checks)} 项通过")
    return 0 if fails == 0 else 1


def cmd_headless(target_height: float, duration: float, outdir: Path,
                 export_docs: bool = False) -> int:
    from mujoco_hopper_slip_control import experiments, figures, metrics

    outdir.mkdir(parents=True, exist_ok=True)

    print("=" * 68)
    print(f"实验 A：定高弹跳（目标 {target_height:.2f} m，{duration:.0f} s）")
    a = experiments.run_height(target_height=target_height, duration=duration)
    st = a.stats
    print(f"  总跳数 {st['hops_total']}（稳态统计取最后 {st['count']} 跳）")
    print(f"  顶点 {st['apex_mean']:.3f} ± {st['apex_std']:.3f} m  误差 {st['apex_error']:.3f} m")
    print(f"  支撑相 {st['stance_mean'] * 1e3:.0f} ms  飞行相 {st['flight_mean'] * 1e3:.0f} ms  "
          f"占空比 {st['duty']:.2f}")
    print(f"  起飞竖直速度 {st['vz_liftoff_mean']:.2f} m/s  峰值腿力 {st['peak_force_mean']:.0f} N")
    metrics.write_csv(outdir / "hops_height.csv", metrics.hops_to_rows(a.hops))
    figures.plot_height(a, outdir / "apex_height.png")
    figures.plot_phase(a, outdir / "phase_cycle.png")

    print("=" * 68)
    print(f"实验 B：抗扰恢复（t={config.DISTURB_T:.0f}s 施加 +{config.DISTURB_DV:.1f} m/s 横向冲量）")
    b = experiments.run_disturbance(target_height=target_height)
    ex = b.extra
    rec = b.stats["recovery_time"]
    print(f"  扰动后 |vx| 峰值 {ex['vx_peak']:.2f} m/s   最低高度 {ex['z_min_after']:.3f} m   "
          f"存活 {bool(ex['survived'])}")
    print(f"  顶点恢复时间 {rec:.2f} s（判定：连续 3 跳顶点回到目标 ±{config.TOL_APEX:.2f} m）")
    print(f"  扰动后顶点均值 {ex['apex_mean_after']:.3f} m   横向漂移 {ex['drift']:.2f} m "
          f"（末速 {ex['vx_final']:.2f} m/s，属已知局限）")
    figures.plot_disturbance(b, outdir / "disturbance.png")

    print("=" * 68)
    print("实验 C：弹簧刚度扫描")
    c = experiments.run_stiffness_sweep(target_height=target_height)
    rows = c.extra["rows"]
    metrics.write_csv(outdir / "stiffness_sweep.csv", rows)
    header = f"  {'k (N/m)':>8} {'顶点 (m)':>9} {'支撑 (ms)':>9} {'飞行 (ms)':>9} {'峰值力 (N)':>10} {'注能 (N)':>9}"
    print(header)
    for r in rows:
        print(f"  {r['stiffness']:>8.0f} {r['apex_mean']:>9.3f} {r['stance_mean'] * 1e3:>9.0f} "
              f"{r['flight_mean'] * 1e3:>9.0f} {r['peak_force']:>10.0f} {r['injected_force']:>9.0f}")
    figures.plot_sweep(rows, outdir / "stiffness_sweep.png")

    print("=" * 68)
    print(f"结果已写入 {outdir}")

    if export_docs:
        import shutil
        docdir = HERE.parent.parent.parent / "docs" / "img" / "humanoid" / config.PACKAGE_NAME
        docdir.mkdir(parents=True, exist_ok=True)
        for name in ("apex_height.png", "phase_cycle.png", "stiffness_sweep.png",
                     "disturbance.png"):
            shutil.copyfile(outdir / name, docdir / name)
        print(f"图表已导出到 {docdir}")

    ok = a.ok and b.ok
    print("结论：" + ("三项实验均跑通" if ok else "存在未达标的实验，见上表"))
    return 0 if ok else 1


def cmd_gif(target_height: float, duration: float, out: Path) -> int:
    from mujoco_hopper_slip_control import figures
    p = figures.render_gif(target_height=target_height, path=out, duration=duration)
    size = p.stat().st_size / 1024
    print(f"动图已生成 {p}  ({size:.0f} KB)")
    return 0 if size < 5 * 1024 else 1


def cmd_gui(target_height: float) -> int:
    import mujoco.viewer
    from mujoco_hopper_slip_control.sim import HopperSim

    sim = HopperSim(target_height=target_height)
    print("窗口已打开，关闭窗口结束。")
    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        while viewer.is_running():
            sim.step()
            viewer.sync()
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="MuJoCo 单足弹跳机器人（SLIP + Raibert）")
    ap.add_argument("--version", action="store_true", help="打印版本")
    ap.add_argument("--test", action="store_true", help="离线自检")
    ap.add_argument("--headless", action="store_true", help="跑实验并出图")
    ap.add_argument("--gif", action="store_true", help="生成演示动图")
    ap.add_argument("--gui", action="store_true", help="交互窗口")
    ap.add_argument("--target-height", type=float, default=config.TARGET_HEIGHT,
                    help=f"顶点目标高度 m（默认 {config.TARGET_HEIGHT}）")
    ap.add_argument("--duration", type=float, default=12.0, help="仿真时长 s")
    ap.add_argument("--outdir", type=str, default="results", help="输出目录")
    ap.add_argument("--export-docs", action="store_true",
                    help="把图表同步到 docs/img/humanoid/<包名>/（写文档时用）")
    ap.add_argument("--gif-out", type=str,
                    default=str(HERE.parent.parent.parent / "docs" / "img" / "humanoid"
                                / config.PACKAGE_NAME / "demo.gif"),
                    help="动图输出路径")
    args = ap.parse_args(argv)

    if args.version:
        return cmd_version()
    if args.test:
        return cmd_test()
    if args.gif:
        return cmd_gif(args.target_height, min(args.duration, 8.0), _real(args.gif_out))
    if args.gui:
        return cmd_gui(args.target_height)
    if args.headless:
        return cmd_headless(args.target_height, args.duration, _real(args.outdir),
                            export_docs=args.export_docs)
    ap.print_help()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
