#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双足步态生成与 ZMP 稳定性验证 —— 演示入口。

运行::

    python3 main.py                # 完整演示
    python3 main.py --quick        # 跳过增益扫描
    python3 main.py --outdir DIR

输出：

* 终端打印 LIPM 参数、开环发散率、闭环跟踪指标
* demo_out/gait_com_zmp.png     质心轨迹与 ZMP
* demo_out/stability_margin.png 稳定裕度随时间变化
* demo_out/dcm_error.png        DCM 跟踪误差
* demo_out/gain_sweep.png       不同反馈增益下的行程

图默认写到**包根**的 demo_out/（不是 scripts/ 下），与 README、launch 的默认
输出目录一致，想换地方用 --outdir 指定。

只用 Python 标准库。
"""

from __future__ import annotations

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from dcm import (  # noqa: E402
    DEFAULT_GAIN,
    dcm,
    summarise_track,
    track,
)
from footstep import stability_margin  # noqa: E402
from gait import (  # noqa: E402
    build_symmetric_plan,
    integrate,
    simulate,
    solve_periodic,
    summarise,
)
from lipm import LIPMParams, com_trajectory  # noqa: E402
from simple_plot import plot_series  # noqa: E402

BLUE = (40, 110, 220)
RED = (215, 60, 55)
GREEN = (35, 165, 90)
ORANGE = (235, 150, 30)
PURPLE = (140, 80, 190)
TEAL = (20, 160, 175)
GRAY = (150, 150, 150)


def hr(title=""):
    line = "=" * 72
    if title:
        print(line)
        print(title)
        print(line)
    else:
        print(line)


def demo_lipm(params):
    """打印 LIPM 基本量，并检查解析解是否满足运动方程。"""
    print()
    hr("LIPM 参数")
    om = params.omega
    print(f"  质心高度 z_c      : {params.z_c:.3f} m")
    print(f"  重力加速度 g      : {params.g:.2f} m/s²")
    print(f"  自然频率 ω        : {om:.4f} 1/s")
    print(f"  特征时间 1/ω      : {1.0 / om:.4f} s")
    print(f"  振荡周期 2π/ω     : {2 * math.pi / om:.4f} s")

    # 解析解校验：数值微分与解析加速度对比
    x0, v0, pz, t = 0.1, 0.4, 0.0, 0.3
    x, v, a = com_trajectory(x0, v0, pz, t, om)
    h = 1e-6
    xp, _, _ = com_trajectory(x0, v0, pz, t + h, om)
    xm, _, _ = com_trajectory(x0, v0, pz, t - h, om)
    a_num = (xp - 2 * x + xm) / (h * h)
    print()
    print("  解析解校验（t=0.3 s）：")
    print(f"    解析加速度      : {a:+.9f}")
    print(f"    数值二阶差分    : {a_num:+.9f}")
    print(f"    差值            : {abs(a - a_num):.2e}")


def demo_instability(plan, params):
    """证明开环周期解是不稳定的，并测出每周期放大倍数。"""
    print()
    hr("开环周期解的不稳定性")
    om = params.omega
    x0, v0, stride, t0 = solve_periodic(plan, params, cycle_steps=2, dt=0.001)
    print(f"  周期起点 t = {t0:.2f} s")
    print(f"  周期初值 x0 = {x0:+.5f} m, v0 = {v0:+.5f} m/s")
    print(f"  每周期前进 stride = {stride:.3f} m")
    print()

    zf = plan.zmp_reference
    x, v = x0, v0
    cycle_rows = []
    for k in range(5):
        ta = t0 + k * 1.0
        x, v = integrate(x, v, zf, ta, ta + 1.0, 0.001, om)
        dv = v - v0
        cycle_rows.append((k + 1, dv))
        print(f"  第 {k + 1} 周期后  速度偏差 Δv = {dv:+.3e}")

    if len(cycle_rows) >= 3 and abs(cycle_rows[0][1]) > 0:
        ratios = [cycle_rows[i + 1][1] / cycle_rows[i][1]
                  for i in range(len(cycle_rows) - 1)
                  if abs(cycle_rows[i][1]) > 1e-15]
        mean_ratio = sum(ratios) / len(ratios)
        print()
        print(f"  相邻周期放大倍数   : {mean_ratio:.2f}")
        print(f"  理论值 e^(ω·T)     : {math.exp(om * 1.0):.2f}   (T = 1.0 s)")
        print()
        print("  这就是 LIPM 的本质：周期解存在，但它是**不稳定平衡点**，")
        print("  任何微小误差都会按 e^(ωt) 放大。所以真实机器人必须闭环控制。")
    return x0, v0, stride, t0


def demo_closed_loop(plan, params, v_avg, gain=DEFAULT_GAIN, outdir=None):
    """DCM 反馈闭环行走。"""
    print()
    hr("闭环行走（DCM 反馈 + ZMP 限幅）")
    print(f"  反馈律     : p = p_ref + {gain}·(ξ − ξ_ref)")
    print(f"  平均速度   : {v_avg:.3f} m/s")
    print()

    traj = track(plan, params, 0.0, 0.0, v_avg, dt=0.001, gain=gain)
    s = summarise_track(traj)

    print(f"  仿真时长          : {s['duration']:.3f} s")
    print(f"  质心行程          : {s['com_travel']:+.4f} m")
    print(f"  平均速度          : {s['com_travel'] / s['duration']:.4f} m/s")
    print(f"  最大速度          : {s['com_max_speed']:.4f} m/s")
    print(f"  DCM 最大误差      : {s['dcm_max_error']:.4f} m")
    print(f"  DCM 平均误差      : {s['dcm_mean_error']:.4f} m")
    print(f"  最小稳定裕度      : {s['min_margin']:+.4f} m")
    print(f"  ZMP 饱和比例      : {s['saturated_ratio'] * 100:.1f} %")
    print(f"  是否全程可行      : {'是' if s['stable'] else '否'}")

    # 轨迹图
    lo = traj["poly_lo"]
    hi = traj["poly_hi"]
    plot_series(
        os.path.join(outdir, "gait_com_zmp.png"),
        [("com x (m)", traj["time"], traj["com_x"], BLUE),
         ("zmp (m)", traj["time"], traj["zmp_actual"], RED),
         ("support hi", traj["time"], hi, GRAY),
         ("support lo", traj["time"], lo, GRAY)],
        title="COM AND ZMP DURING WALKING",
    )

    plot_series(
        os.path.join(outdir, "stability_margin.png"),
        [("margin (m)", traj["time"], traj["margin"], GREEN)],
        title="ZMP STABILITY MARGIN",
    )

    errs = [abs(a - b) for a, b in zip(traj["dcm"], traj["dcm_ref"])]
    plot_series(
        os.path.join(outdir, "dcm_error.png"),
        [("dcm error (m)", traj["time"], errs, PURPLE)],
        title="DCM TRACKING ERROR",
    )
    print()
    print(f"  曲线已保存      : {outdir}/gait_com_zmp.png, "
          f"stability_margin.png, dcm_error.png")
    return traj, s


def demo_gain_sweep(plan, params, v_avg, outdir):
    """扫描反馈增益，看什么时候能走起来。"""
    print()
    hr("反馈增益扫描")
    print(f"  {'增益 K':>8} {'行程(m)':>12} {'最大速度':>12} "
          f"{'饱和比例':>10} {'平均裕度':>10}")
    print("  " + "-" * 60)

    gains = [0.0, 0.3, 0.6, 1.0, 1.5, 2.0, 3.0]
    rows = []
    for g in gains:
        traj = track(plan, params, 0.0, 0.0, v_avg, dt=0.002, gain=g)
        s = summarise_track(traj)
        mean_margin = sum(traj["margin"]) / len(traj["margin"])
        rows.append((g, s["com_travel"], s["com_max_speed"],
                     s["saturated_ratio"], mean_margin))
        print(f"  {g:>8.1f} {s['com_travel']:>12.4f} {s['com_max_speed']:>12.4f} "
              f"{s['saturated_ratio'] * 100:>9.1f}% {mean_margin:>10.4f}")

    print()
    print("  K = 0（纯开环跟踪参考）会发散，K ≥ 1 才能稳定行走。")
    print("  K 越大 ZMP 越常贴到支撑边界（饱和比例升高），裕度反而变小。")

    # 用对数刻度不合适，这里直接画行程的绝对值
    plot_series(
        os.path.join(outdir, "gain_sweep.png"),
        [("travel (m)", [g for g, *_ in rows],
          [abs(t) for _g, t, *_ in rows], BLUE)],
        title="TRAVEL VS FEEDBACK GAIN",
    )
    return rows


def demo_plan_table(plan):
    """打印落脚点与支撑区间。"""
    print()
    hr("步态计划")
    print(f"  {'脚':>3} {'中心(m)':>10} {'着地(s)':>9} {'离地(s)':>9} "
          f"{'支撑区间':>22}")
    print("  " + "-" * 60)
    for s in plan.steps[:8]:
        lo, hi = s.reach
        print(f"  {s.side:>3} {s.center:>10.3f} {s.t_start:>9.2f} "
              f"{s.t_end:>9.2f}   [{lo:+.3f}, {hi:+.3f}]")
    if len(plan.steps) > 8:
        print(f"  ... 共 {len(plan.steps)} 步")


def main():
    ap = argparse.ArgumentParser(description="双足步态生成与 ZMP 稳定性验证")
    ap.add_argument("--outdir",
                    default=os.path.normpath(os.path.join(HERE, os.pardir, "demo_out")),
                    help="曲线输出目录（默认包根 demo_out/）")
    ap.add_argument("--quick", action="store_true", help="跳过增益扫描")
    ap.add_argument("--gain", type=float, default=DEFAULT_GAIN,
                    help="DCM 反馈增益（默认 1.0）")
    # roslaunch 启动 <node> 时会在命令行末尾追加 __name:= / __log:=，
    # 用 parse_known_args 忽略掉，否则 launch 一启动就退出。
    args, _unknown = ap.parse_known_args()

    if not os.path.isdir(args.outdir):
        os.makedirs(args.outdir)

    params = LIPMParams(z_c=0.8)
    step_length, step_time, duty = 0.35, 0.5, 0.6
    v_avg = step_length / step_time

    print()
    hr("双足步态生成与 ZMP 稳定性验证")
    print()
    print("  思路：把机器人简化成线性倒立摆（LIPM），由落脚点计划给出 ZMP")
    print("  参考轨迹，先求开环周期解，再说明它为什么不稳定、怎样闭环稳住。")

    demo_lipm(params)

    plan = build_symmetric_plan(n_steps=12, step_length=step_length,
                                step_time=step_time, duty=duty)
    print()
    print(f"  步长 {step_length} m，步时 {step_time} s，支撑相占比 {duty}")
    print(f"  → 平均速度 {v_avg:.2f} m/s")
    demo_plan_table(plan)

    demo_instability(plan, params)
    demo_closed_loop(plan, params, v_avg, gain=args.gain, outdir=args.outdir)
    if not args.quick:
        demo_gain_sweep(plan, params, v_avg, args.outdir)

    print()
    hr("完成")
    print(f"  图片目录: {os.path.abspath(args.outdir)}")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
