#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""肌肉驱动的单自由度关节控制 —— 演示入口。

运行::

    python3 main.py                # 完整演示，输出到 demo_out/
    python3 main.py --quick        # 跳过参数扫描，只跑基本阶跃响应
    python3 main.py --outdir DIR   # 指定输出目录

输出：

* 终端打印的模型参数与控制性能指标
* ``demo_out/joint_step.png``     关节角跟踪曲线
* ``demo_out/activation.png``     两块肌肉的激活动作
* ``demo_out/muscle_force.png``   肌肉力与关节力矩
* ``demo_out/coactivation.png``   不同共激活水平下的跟踪对比

本模块只依赖标准库，不需要 MuJoCo / ROS / matplotlib。
"""

from __future__ import annotations

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from joint_model import (  # noqa: E402
    JointParams,
    MuscleJoint,
    evaluate,
    simulate_step_response,
)
from muscle_model import (  # noqa: E402
    force_length_active,
    force_length_passive,
    force_velocity,
    MuscleParams,
)
from simple_plot import plot_series  # noqa: E402
from stiffness import (  # noqa: E402
    disturbance_response,
    joint_stiffness,
    operating_segment,
    slope_balance,
    stiffness_sweep,
    tradeoff_analysis,
)

BLUE = (40, 110, 220)
RED = (215, 60, 55)
GREEN = (35, 165, 90)
ORANGE = (235, 150, 30)
PURPLE = (140, 80, 190)
TEAL = (20, 160, 175)


def hr(title=""):
    line = "=" * 72
    if title:
        print(line)
        print(title)
        print(line)
    else:
        print(line)


def print_params(p):
    print(f"  转动惯量 J        : {p.inertia:.4f} kg·m²")
    print(f"  关节阻尼 b        : {p.damping:.4f} N·m·s/rad")
    print(f"  力矩臂 r          : {p.moment_arm:.4f} m")
    print(f"  肌肉静息长度      : {p.rest_length:.4f} m")
    print(f"  肌纤维最优长度    : {p.l_opt:.4f} m")
    print(f"  最大等长收缩力    : {p.f_max:.1f} N  (单块)")
    print(f"  最大缩短速度      : {p.v_max:.2f} m/s")
    print(f"  激活时间常数      : {p.tau_act:.4f} s")
    print(f"  放松时间常数      : {p.tau_deact:.4f} s")
    print(f"  最大关节力矩      : {p.f_max * p.moment_arm:.4f} N·m"
          f"  (= f_max · r)")


def print_metrics(m):
    rt = m["rise_time"]
    print(f"  目标角度          : 见上")
    print(f"  最终角度          : {m['final_angle']:+.6f} rad")
    print(f"  稳态误差          : {m['steady_state_error']:+.6f} rad")
    print(f"  上升时间(90%)     : "
          + (f"{rt:.3f} s" if rt is not None else "未达到"))
    print(f"  峰值角度          : {m['peak_angle']:+.6f} rad")
    print(f"  超调量            : {m['overshoot_percent']:.2f} %")
    print(f"  屈肌最大激活度    : {m['max_activation_flex']:.4f}")
    print(f"  伸肌最大激活度    : {m['max_activation_ext']:.4f}")
    print(f"  最大关节力矩      : {m['max_torque']:.4f} N·m")
    print(f"  仿真步数          : {m['steps']}")


# ---------------------------------------------------------------------------
def demo_muscle_curves(outdir):
    """画出三条肌肉特性曲线，便于直观核对模型。"""
    print()
    hr("肌肉特性曲线")
    p = MuscleParams()

    l_grid = [0.5 + i * 0.01 for i in range(101)]          # 0.5 ~ 1.5
    fl = [force_length_active(l, p.width) for l in l_grid]
    fp = [force_length_passive(l) for l in l_grid]

    pdf = plot_series(
        os.path.join(outdir, "muscle_curves.png"),
        [("active f_l", l_grid, fl, BLUE),
         ("passive f_p", l_grid, fp, RED)],
        title="MUSCLE FORCE-LENGTH",
    )
    print(f"  力-长度曲线      -> {pdf}")
    print(f"    主动力峰值出现在 l_norm = "
          f"{l_grid[fl.index(max(fl))]:.2f}，值 {max(fl):.3f}")

    v_grid = [-0.8 + i * 0.008 for i in range(226)]        # -0.8 ~ 1.0
    fv = [force_velocity(v) for v in v_grid]
    pdf = plot_series(
        os.path.join(outdir, "force_velocity.png"),
        [("f_v", v_grid, fv, GREEN)],
        title="MUSCLE FORCE-VELOCITY",
    )
    print(f"  力-速度曲线      -> {pdf}")
    print(f"    等长(v=0) 力  = {force_velocity(0.0):.4f}")
    print(f"    离心(v=-0.5)  = {force_velocity(-0.5):.4f}  (>1，离心更强)")
    print(f"    最大缩短(v=1) = {force_velocity(1.0):.4f}")
    return True


def demo_step_response(outdir, target=0.35, duration=1.5, dt=0.001,
                       kp=9.0, kd=0.9, co=0.02):
    """基本阶跃响应。"""
    print()
    hr(f"阶跃响应：目标关节角 {target} rad")
    res = simulate_step_response(target, duration=duration, dt=dt,
                                 kp=kp, kd=kd, co_activation=co)
    h, m = res["history"], res["metrics"]
    print(f"  控制器            : PD, kp={kp}, kd={kd}, 共激活={co}")
    print_metrics(m)

    # 图 1：关节角跟踪
    plot_series(
        os.path.join(outdir, "joint_step.png"),
        [("theta (rad)", h["t"], h["theta"], BLUE),
         ("target", h["t"], [target] * len(h["t"]), RED)],
        title="JOINT ANGLE TRACKING",
    )

    # 图 2：激活动作
    plot_series(
        os.path.join(outdir, "activation.png"),
        [("flexor a", h["t"], h["a_flex"], RED),
         ("extensor a", h["t"], h["a_ext"], BLUE)],
        title="MUSCLE ACTIVATION",
    )

    # 图 3：肌肉力与关节力矩
    plot_series(
        os.path.join(outdir, "muscle_force.png"),
        [("flexor F (N)", h["t"], h["f_flex"], RED),
         ("extensor F (N)", h["t"], h["f_ext"], BLUE),
         ("joint torque x100 (Nm)", h["t"],
          [x * 100.0 for x in h["tau"]], GREEN)],
        title="MUSCLE FORCE / JOINT TORQUE",
    )
    print(f"  曲线已保存到      : {outdir}/joint_step.png, "
          f"activation.png, muscle_force.png")
    return res


def demo_coactivation(outdir, target=0.35, duration=1.2, dt=0.001):
    """对比不同共激活水平的影响：共激活越高，关节越"僵"。"""
    print()
    hr("共激活水平对比")
    levels = [0.02, 0.15, 0.35]
    series_theta = []
    series_torque = []
    colors = [BLUE, ORANGE, PURPLE]
    print(f"  {'共激活':>8}  {'稳态误差(rad)':>14}  {'上升时间(s)':>12}  "
          f"{'超调(%)':>9}")
    for lvl, col in zip(levels, colors):
        res = simulate_step_response(target, duration=duration, dt=dt,
                                     co_activation=lvl)
        h, m = res["history"], res["metrics"]
        series_theta.append((f"co={lvl:.2f}", h["t"], h["theta"], col))
        series_torque.append((f"co={lvl:.2f}", h["t"], h["tau"], col))
        rt = m["rise_time"]
        print(f"  {lvl:>8.2f}  {m['steady_state_error']:>14.6f}  "
              f"{(rt if rt is not None else float('nan')):>12.3f}  "
              f"{m['overshoot_percent']:>9.2f}")

    plot_series(os.path.join(outdir, "coactivation.png"), series_theta,
                title="CO-ACTIVATION EFFECT ON TRACKING")
    print(f"  对比图已保存      : {outdir}/coactivation.png")


def demo_torque_limit(outdir, duration=1.5, dt=0.001):
    """说明肌肉驱动与力矩驱动的差异：指令超出肌肉能力时会被饱和。"""
    print()
    hr("力矩饱和现象（肌肉驱动的固有限制）")
    p = JointParams()
    tau_max = p.f_max * p.moment_arm
    print(f"  单块肌肉全力时的关节力矩上限 : {tau_max:.4f} N·m")
    print()
    print(f"  {'目标(rad)':>10}  {'所需力矩(Nm)':>13}  {'实际最大(Nm)':>13}  "
          f"{'稳态误差':>11}")
    for target in (0.1, 0.3, 0.6, 1.0):
        kp = 9.0
        res = simulate_step_response(target, duration=duration, dt=dt, kp=kp)
        m = res["metrics"]
        need = kp * target
        print(f"  {target:>10.2f}  {need:>13.4f}  {m['max_torque']:>13.4f}  "
              f"{m['steady_state_error']:>11.5f}")
    print()
    print("  目标越大，PD 要求的力矩越可能超过肌肉上限，误差随之增大——")
    print("  这是肌肉驱动与理想力矩源的差别，本模块把它如实体现出来。")


def demo_stiffness(outdir):
    """共激活对关节刚度的影响 —— 取决于肌肉工作在力-长度曲线的哪一段。"""
    print()
    hr("拮抗肌共激活与关节刚度")
    print()
    print("  刚度定义：k = -∂τ/∂θ，正值表示恢复力矩（稳定）")
    print("  由 k = r²·(∂F_flex/∂l + ∂F_ext/∂l) 可知，刚度正比于两条肌肉的")
    print("  力-长度曲线斜率之和。因此共激活的效应取决于工作点在哪一段。")
    print()

    cases = [
        (0.95, "上升段", BLUE),
        (1.00, "最优长度", GREEN),
        (1.05, "下降段", RED),
    ]
    co_levels = [0.05, 0.1, 0.2, 0.3, 0.45, 0.6, 0.75, 0.9]

    print(f"  {'l_rest/l_opt':>13} {'工作段':>10} {'k(co=0.05)':>13} "
          f"{'k(co=0.9)':>12} {'趋势':>9}")
    series = []
    for ratio, seg, color in cases:
        p = JointParams(rest_length=0.10 * ratio, l_opt=0.10)
        rows = stiffness_sweep(p, co_levels)
        ks = [k for _c, k in rows]
        if abs(ks[-1] - ks[0]) < 1e-6:
            trend = "不变"
        else:
            trend = "上升" if ks[-1] > ks[0] else "下降"
        print(f"  {ratio:>13.2f} {seg:>10} {ks[0]:>13.4f} {ks[-1]:>12.4f} "
              f"{trend:>9}")
        series.append((f"{seg} ({ratio:.2f})", co_levels, ks, color))

    plot_series(os.path.join(outdir, "stiffness_vs_coactivation.png"), series,
                title="JOINT STIFFNESS VS CO-ACTIVATION")
    print()
    print("  结论：只有**上升段**上「共激活提高刚度」才成立。")
    print("  下降段上拉伸肌肉反而让它主动力变小（负弹簧），激活越高刚度越低，")
    print("  越过被动力后就失稳。下面看机理。")

    print()
    print("  机理解析（以下降段 l_rest/l_opt = 1.05 为例）")
    print(f"  {'激活 a':>8} {'a·df_l/dl':>12} {'df_p/dl':>11} {'合计':>11} "
          f"{'是否稳定':>10}")
    p = JointParams(rest_length=0.105, l_opt=0.10)
    for a in (0.0, 0.2, 0.5, 0.8, 1.0):
        b = slope_balance(p, a)
        print(f"  {a:>8.1f} {b['active']:>12.4f} {b['passive']:>11.4f} "
              f"{b['total']:>11.4f} {'稳定' if b['stable'] else '失稳':>10}")

    print()
    print("  刚度随关节角的变化（三个工作点，共激活 0.3）")
    print(f"  {'θ (rad)':>10} " + "".join(f"{s:>14}" for _r, s, _c in cases))
    theta_grid = [-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3]
    series_k = []
    for ratio, seg, color in cases:
        p = JointParams(rest_length=0.10 * ratio, l_opt=0.10)
        ks = [joint_stiffness(p, th, 0.3, 0.3) for th in theta_grid]
        series_k.append((seg, theta_grid, ks, color))
    for i, th in enumerate(theta_grid):
        # series_k 的元素是 (标签, θ 网格, 刚度序列, 颜色)，刚度在索引 2
        print(f"  {th:>10.2f} " +
              "".join(f"{s[2][i]:>14.4f}" for s in series_k))

    plot_series(os.path.join(outdir, "stiffness_vs_angle.png"), series_k,
                title="STIFFNESS VS JOINT ANGLE (CO=0.3)")

    # 扰动响应
    print()
    print("  抗扰测试：施加 0.05 N·m 恒定外力矩，测稳态偏转（上升段 co 扫描）")
    print(f"  {'共激活':>8} {'稳态偏转(rad)':>15} {'等效刚度(Nm/rad)':>18} "
          f"{'内置刚度':>11}")
    p = JointParams(rest_length=0.095, l_opt=0.10)
    colors = [BLUE, GREEN, ORANGE, RED]
    series_def = []
    for idx, co in enumerate((0.05, 0.2, 0.5, 0.8)):
        r = disturbance_response(p, co=co, tau_ext=0.05, duration=1.5)
        k_in = joint_stiffness(p, 0.0, co, co)
        print(f"  {co:>8.2f} {r['theta_ss']:>15.6f} "
              f"{r['k_equivalent']:>18.4f} {k_in:>11.4f}")
        series_def.append((f"co={co:.2f}", r["history"]["t"],
                           r["history"]["theta"], colors[idx]))
    plot_series(os.path.join(outdir, "disturbance_response.png"), series_def,
                title="DEFLECTION UNDER 0.05 Nm DISTURBANCE")
    print()
    print("  上升段上共激活越大，同样扰动下的偏转越小 —— 刚度确实提高了。")
    print(f"  曲线已保存      : {outdir}/stiffness_vs_coactivation.png, "
          f"stiffness_vs_angle.png, disturbance_response.png")


def demo_tradeoff(outdir):
    """刚度-精度-能耗的三角权衡。"""
    print()
    hr("刚度 - 精度 - 能耗的权衡（上升段 l_rest = 0.95·l_opt）")
    p = JointParams(rest_length=0.095, l_opt=0.10)
    levels = [0.02, 0.1, 0.2, 0.35, 0.5]
    rows = tradeoff_analysis(p, levels, duration=2.5, dt=0.001, target=0.3)

    print(f"  {'共激活':>8} {'刚度 k':>10} {'稳态误差':>12} {'上升时间':>10} "
          f"{'超调%':>8} {'能耗代理':>10}")
    for r in rows:
        rt = r["rise_time"]
        print(f"  {r['co']:>8.2f} {r['k']:>10.4f} {r['ss_error']:>12.6f} "
              f"{(rt if rt is not None else float('nan')):>10.3f} "
              f"{r['overshoot']:>8.2f} {r['energy']:>10.3f}")

    # 权衡图：横轴用刚度，纵轴用误差绝对值
    series = [
        ("|steady error| (rad)", [r["k"] for r in rows],
         [abs(r["ss_error"]) for r in rows], RED),
    ]
    plot_series(os.path.join(outdir, "tradeoff.png"), series,
                title="STIFFNESS VS ACCURACY TRADEOFF")
    print()
    print("  刚度越大，误差也越大；能耗同时线性上升。")
    print("  这就是共激活的代价 —— 换来抗扰能力，牺牲定位精度与能耗。")
    print(f"  权衡图已保存    : {outdir}/tradeoff.png")


def demo_operating_points(outdir):
    """三种工作点下的刚度-关节角曲线对比（含"共激活提高刚度"的适用范围）。"""
    print()
    hr("工作点对刚度的影响范围")
    print()
    print("  同一个关节，只因为肌肉静止长度相对最优长度不同，")
    print("  共激活对刚度的作用方向就完全相反：")
    print()
    for ratio in (0.90, 0.95, 0.98, 1.00, 1.02, 1.05, 1.10):
        p = JointParams(rest_length=0.10 * ratio, l_opt=0.10)
        seg = operating_segment(p)
        k_low = joint_stiffness(p, 0.0, 0.05, 0.05)
        k_high = joint_stiffness(p, 0.0, 0.90, 0.90)
        if abs(k_high - k_low) < 1e-6:
            arrow = "="
        else:
            arrow = "↑" if k_high > k_low else "↓"
        print(f"  l_rest/l_opt = {ratio:>4.2f}  {seg:>8}  "
              f"k: {k_low:>8.4f} → {k_high:>8.4f}  {arrow}")


def main():
    ap = argparse.ArgumentParser(description="肌肉驱动的单自由度关节控制演示")
    ap.add_argument("--outdir", default=os.path.join(HERE, "demo_out"),
                    help="图片输出目录")
    ap.add_argument("--quick", action="store_true", help="只跑基本阶跃响应")
    # roslaunch 启动 <node> 时会在命令行末尾追加 __name:= / __log:=，
    # 用 parse_known_args 忽略掉，否则 launch 一启动就退出。
    args, _unknown = ap.parse_known_args()

    if not os.path.isdir(args.outdir):
        os.makedirs(args.outdir)

    print()
    hr("肌肉驱动的单自由度关节控制 —— Hill 模型演示")
    print()
    print("模型说明：关节两侧各连一块肌肉（屈肌/伸肌），控制器输出的不是力矩，")
    print("而是肌肉的激活度 a ∈ [0,1]，力由 Hill 模型根据当前长度与速度算出。")
    print()
    print("关节与肌肉参数：")
    print_params(JointParams())

    demo_muscle_curves(args.outdir)
    demo_step_response(args.outdir)
    if not args.quick:
        demo_coactivation(args.outdir)
        demo_torque_limit(args.outdir)
        demo_stiffness(args.outdir)
        demo_tradeoff(args.outdir)
        demo_operating_points(args.outdir)

    print()
    hr("完成")
    print(f"  所有图片存放在: {os.path.abspath(args.outdir)}")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
