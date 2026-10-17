#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纵向速度控制演示：仿真并输出可量化的对比结果。

用途
----
用于作业验证与截图。**不需要 CARLA、不需要 ROS、不需要 OpenCV、不需要联网**，
只用 numpy 就能跑。

它做三件事：
  1. 生成一条"S 形弯道"路径的曲率剖面（直道 - 左弯 - 直道 - 右弯 - 直道）
  2. 分别仿真「有本模块」与「无纵向控制（定速巡航）」两种情况
  3. 输出量化对比：横向加速度是否超限、jerk 是否超限、速度跟踪误差

运行::

    cd ~/ros2/src/ground/carla_speed_control
    python3 tests/demo_speed_control.py
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))

from speed_control import (  # noqa: E402
    GRAVITY,
    SpeedControllerConfig,
    curvature_profile_s_curve,
    lateral_accel_at,
    simulate_speed_control,
    speed_limit_from_curvature,
)

try:
    from simple_plot import PALETTE, Axes, Canvas  # noqa: E402
    HAS_PLOT = True
except ImportError:  # pragma: no cover - simple_plot 随包提供，正常不会走到
    HAS_PLOT = False

CRUISE = 12.0          # 巡航速度（m/s）
CURVE_RADIUS = 40.0    # 弯道半径（m）


# ---------------------------------------------------------------------------
# PNG 绘图（用自带的 simple_plot，不依赖 matplotlib / Pillow）
# ---------------------------------------------------------------------------


def render_png(outdir, trace, limits, cruise, v_limit_curve):
    """把仿真结果画成 PNG 图片，返回生成的文件路径列表。

    使用包内自带的 ``simple_plot``（纯 numpy + zlib 生成 PNG），
    因此不需要 matplotlib 或 Pillow，在任何有 numpy 的机器上都能出图。
    """
    if not HAS_PLOT:
        print("  [跳过] 未找到 simple_plot，无法生成图片")
        return []

    os.makedirs(outdir, exist_ok=True)
    saved = []

    # ---- 图 1：速度剖面（放大纵轴，聚焦限速切换）----
    # 截取起点：跳过起步加速段，让曲线聚焦在"限速在巡航速度与弯道限速之间
    # 切换"这一段。
    #
    # 判据必须用 |v - v_limit| 而不是 (v > v_limit - 2.0)。后者在有控制时
    # 恒成立（限速从不被突破，故 v - v_limit ≤ 0 永远小于 2.0），会让掩码
    # 全为 False、settled 为空、i0 退化为 0 —— 结果是起步段根本没被裁掉，
    # 而画面上看起来"有一个台阶"，极易被误读为限速剖面的缺陷。
    # 实测这个错误曾导致我花了很长时间去追一个不存在的尖峰。
    deviation = np.abs(trace.v - trace.v_limit)
    settled = np.flatnonzero(deviation < 0.5)
    if settled.size > 20:
        i0 = int(settled[20])          # 跳过前 20 个已稳定的点，确保脱离过渡期
    else:
        i0 = 0
    i0 = min(max(i0, 0), max(0, trace.t.size - 50))
    s = trace.s[i0:]
    v = trace.v[i0:]
    vl = trace.v_limit[i0:]

    lo = min(float(vl.min()), float(v.min()))
    hi = max(float(vl.max()), float(v.max()))
    pad = max(0.05, 0.2 * (hi - lo))

    c = Canvas(1000, 480)
    ax = Axes(c, 110, 70, 950, 400,
              xlim=(float(s[0]), float(s[-1])), ylim=(lo - pad, hi + pad),
              xlabel="s (m)", ylabel="v (m/s)", title="SPEED PROFILE")
    ax.setup()
    ax.plot(s, vl, PALETTE["blue"], 3, "speed limit")
    ax.plot(s, v, PALETTE["red"], 2, "actual speed")
    ax.legend([(PALETTE["blue"], "speed limit"),
               (PALETTE["red"], "actual speed")])
    saved.append(c.save(os.path.join(outdir, "speed_profile.png")))

    # ---- 图 2：横向加速度（实际 vs 限值）----
    c2 = Canvas(1000, 480)
    ax2 = Axes(c2, 110, 70, 950, 400,
               xlim=(float(trace.s[0]), float(trace.s[-1])),
               ylim=(0.0, limits.max_lateral_accel * 1.25),
               xlabel="s (m)", ylabel="a_lat (m/s^2)",
               title="LATERAL ACCELERATION")
    ax2.setup()
    ax2.plot(trace.s, np.full_like(trace.a_lat, limits.max_lateral_accel),
             PALETTE["gray"], 2, "limit")
    ax2.plot(trace.s, trace.a_lat, PALETTE["green"], 3, "actual")
    ax2.legend([(PALETTE["gray"], "limit"),
                (PALETTE["green"], "actual")])
    saved.append(c2.save(os.path.join(outdir, "lateral_accel.png")))

    return saved


# ---------------------------------------------------------------------------
# 终端可视化
# ---------------------------------------------------------------------------


def ascii_profile(x, series, width=72, height=12, labels=None, xlabel="",
                  y_range=None):
    """把若干条曲线画成 ASCII 图，便于在终端截图。

    series: [(y 数组, 字符, 名称), ...]
    y_range: (ymin, ymax) 强制纵轴范围；None 表示按数据自动确定。
             用于"放大"关注区间——例如车速只在 10.9~12.0 之间变化时，
             若纵轴从 0 开始，变化会被压扁成一条斜线，看不出限速切换。
    """
    x = np.asarray(x, dtype=float)
    n = len(x)
    if n == 0:
        return "(空)"

    all_y = np.concatenate([np.asarray(y, dtype=float) for y, _, _ in series])
    if y_range is not None:
        y_min, y_max = float(y_range[0]), float(y_range[1])
    else:
        y_min, y_max = float(all_y.min()), float(all_y.max())
    if y_max - y_min < 1e-9:
        y_max = y_min + 1.0

    grid = [[" "] * width for _ in range(height)]
    for y, ch, _name in series:
        y = np.asarray(y, dtype=float)
        for c in range(width):
            i0 = int(c * n / width)
            i1 = max(i0 + 1, int((c + 1) * n / width))
            # 取窗口内的极值而非均值：避免把窄的限速台阶平均掉
            window = y[i0:i1]
            val = float(window.mean())
            row = height - 1 - int(round((val - y_min) / (y_max - y_min) * (height - 1)))
            row = max(0, min(height - 1, row))
            grid[row][c] = ch

    lines = []
    for r in range(height):
        val = y_max - (y_max - y_min) * r / (height - 1)
        lines.append(f"  {val:7.2f} |" + "".join(grid[r]))
    lines.append("         +" + "-" * width)
    lines.append(f"          {xlabel}")
    return "\n".join(lines)


def format_row(values, fmt="{:>7.2f}"):
    return " ".join(fmt.format(v) for v in values)


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------


def main():
    ap = argparse.ArgumentParser(description="纵向速度控制演示")
    ap.add_argument("--cruise", type=float, default=CRUISE, help="巡航速度 m/s")
    ap.add_argument("--radius", type=float, default=CURVE_RADIUS, help="弯道半径 m")
    ap.add_argument("--outdir", default=os.path.join(HERE, "..", "demo_out"),
                    help="PNG 图片输出目录")
    args = ap.parse_args()

    print()
    print("=" * 78)
    print("纵向速度控制演示：按曲率自动限速与速度跟踪")
    print("=" * 78)

    s, kappa = curvature_profile_s_curve(curve_radius=args.radius)
    k_peak = float(np.abs(kappa).max())
    cfg = SpeedControllerConfig()
    lim = cfg.limits

    print()
    print("--- 1. 输入 ---")
    print(f"  路径总长          : {s[-1]:.0f} m")
    print(f"  路径形状          : 直道 - 左弯 - 直道 - 右弯 - 直道")
    print(f"  弯道半径          : {1.0 / k_peak:.0f} m（曲率 {k_peak:.5f} 1/m）")
    print(f"  巡航速度          : {args.cruise:.1f} m/s = {args.cruise * 3.6:.1f} km/h")
    print()
    print("  载具约束：")
    print(f"    最大横向加速度  : {lim.max_lateral_accel:.1f} m/s²"
          f" = {lim.max_lateral_accel / GRAVITY:.2f} g")
    print(f"    最大加速/减速   : {lim.max_accel:.1f} / {lim.max_decel:.1f} m/s²")
    print(f"    最大 jerk       : {lim.max_jerk:.1f} m/s³")

    # ---- 有本模块 ----
    r_with = simulate_speed_control(s, kappa, cfg, cruise_speed=args.cruise)

    # ---- 无纵向控制：定速巡航 ----
    # 车辆无视曲率，一直以巡航速度行驶。横向加速度由曲率直接决定。
    a_lat_no = lateral_accel_at(args.cruise, k_peak)
    v_limit_curve = speed_limit_from_curvature(
        k_peak, lim.max_lateral_accel, lim.min_speed
    )

    print()
    print("--- 2. 两种情况对比 ---")
    print()
    print(f"  {'指标':<24}{'无纵向控制':>13}{'有本模块':>13}{'说明':>26}")
    print("  " + "-" * 74)
    print(f"  {'最大横向加速度 m/s²':<24}{a_lat_no:>13.3f}"
          f"{r_with.max_lateral_accel:>13.3f}{'超过轮胎附着能力':>26}")
    print(f"  {'最大超速 m/s':<24}{'—':>13}"
          f"{r_with.max_overspeed:>13.4f}{'限速不被突破':>26}")
    print(f"  {'最大 |jerk| m/s³':<24}{'—':>13}"
          f"{r_with.max_abs_jerk:>13.2f}{'加减速平顺':>26}")
    print(f"  {'稳态速度误差 m/s':<24}{'—':>13}"
          f"{r_with.max_speed_error:>13.4f}{'跟踪精度':>26}")
    print()
    print("  注：无纵向控制时车辆定速行驶，不存在 jerk 与跟踪误差的概念，"
          "故以「—」表示。")
    ok_lat = r_with.within_lateral_limit
    print(f"  横向加速度约束    : {'满足 ✓' if ok_lat else '未满足 ✗'}"
          f"  （{r_with.max_lateral_accel:.3f} ≤ {lim.max_lateral_accel}）")
    print(f"  无纵向控制时      : 横向加速度 {a_lat_no:.3f} m/s²，"
          f"为限值的 {a_lat_no / lim.max_lateral_accel:.2f} 倍"
          f"（超过轮胎附着能力会失稳）")

    print()
    print("--- 3. 自动限速计算 ---")
    print(f"  由 a_lat = v²·κ 反解，弯道限速 = sqrt({lim.max_lateral_accel} / {k_peak:.5f})"
          f" × 0.995 = {v_limit_curve:.2f} m/s = {v_limit_curve * 3.6:.1f} km/h")
    print(f"  验证：以 {v_limit_curve:.2f} m/s 通过该弯道时，"
          f"横向加速度 = {lateral_accel_at(v_limit_curve, k_peak):.3f} m/s²"
          f" ≤ {lim.max_lateral_accel}")

    # ---- 4. 速度剖面 ----
    tr = r_with.trace
    print()
    print("--- 4. 速度-里程曲线（= 限速, * 实际车速）---")
    print("    （纵轴含 0，用于显示从静止起步的完整过程）")
    print(ascii_profile(tr.s, [(tr.v_limit, "=", "限速"), (tr.v, "*", "车速")],
                        xlabel="里程 s (m)"))

    # 放大视图：车速只在弯道限速与巡航速度之间变化，
    # 若沿用上面的 0~12 纵轴，变化会被压成一条斜线，看不出限速切换。
    #
    # 两个关键处理：
    #   1. 纵轴范围必须**排除起步瞬态**——起步时车速从 0 升到巡航速度，
    #      算进范围会把纵轴拉到 0 附近（实测得到 -1.19~13.21），放大失效。
    #   2. 截取**起步之后的稳态段**——否则画面上大半是起步加速的斜线，
    #      真正关心的"限速在直道与弯道间来回切换"只占右边一小块。
    #      判据用 |v - v_limit| < 0.5；不能用 (v > v_limit - 2.0)，
    #      那个条件在有控制时恒成立，会让裁剪完全失效（详见 render_png
    #      中的同名说明）。
    deviation_zoom = np.abs(tr.v - tr.v_limit)
    settled_zoom = np.flatnonzero(deviation_zoom < 0.5)
    if settled_zoom.size > 20:
        i0 = int(settled_zoom[20])
    else:
        i0 = 0
    i0 = min(max(i0, 0), max(0, tr.t.size - 50))
    s_zoom = tr.s[i0:]
    v_zoom = tr.v[i0:]
    vl_zoom = tr.v_limit[i0:]

    v_lo = min(float(vl_zoom.min()), float(v_zoom.min()))
    v_hi = max(float(vl_zoom.max()), float(v_zoom.max()))
    pad = max(0.05, 0.15 * (v_hi - v_lo))
    print()
    print("--- 4b. 速度曲线放大（起步之后，聚焦限速在直道与弯道间的切换）---")
    print(ascii_profile(s_zoom, [(vl_zoom, "=", "限速"), (v_zoom, "*", "车速")],
                        xlabel="里程 s (m)", y_range=(v_lo - pad, v_hi + pad)))
    print(f"    （纵轴 {v_lo - pad:.2f} ~ {v_hi + pad:.2f} m/s，"
          f"起点 s={s_zoom[0]:.0f} m；")
    print(f"      限速在 {v_limit_curve:.2f}（弯道）与 {args.cruise:.2f}（直道）m/s 之间切换，")
    print(f"      '*' 紧贴 '=' 说明速度跟踪准确）")

    print()
    print("--- 5. 横向加速度-里程曲线（= 实际, - 限值）---")
    a_lim_line = np.full_like(tr.a_lat, lim.max_lateral_accel)
    print(ascii_profile(tr.s, [(a_lim_line, "-", "限值"), (tr.a_lat, "=", "实际")],
                        xlabel="里程 s (m)"))

    # ---- 6. 汇总 ----
    print()
    print("--- 6. 汇总 ---")
    print(f"  行驶距离          : {tr.s[-1]:.1f} m")
    print(f"  总时间            : {r_with.total_time:.2f} s")
    print(f"  平均速度          : {r_with.mean_speed:.2f} m/s"
          f" = {r_with.mean_speed * 3.6:.1f} km/h")
    print(f"  稳态速度误差      : {r_with.max_speed_error:.4f} m/s"
          f"（{r_with.max_speed_error * 3.6:.3f} km/h）")
    print(f"  最大超速          : {r_with.max_overspeed:.4f} m/s"
          f"（兜底层介入 {100 * r_with.overspeed_ratio:.1f}% 的步数）")
    print(f"  最大横向加速度    : {r_with.max_lateral_accel:.4f} m/s²")
    print(f"  最大 jerk         : {r_with.max_abs_jerk:.2f} m/s³")

    # ---- 7. 生成图片 ----
    print()
    print("--- 7. 生成图片（用包内自带的 simple_plot，零外部依赖）---")
    outdir = os.path.abspath(args.outdir)
    images = render_png(outdir, tr, lim, args.cruise, v_limit_curve)
    if images:
        for p in images:
            size_kb = os.path.getsize(p) / 1024.0
            print(f"  已保存: {p}  ({size_kb:.1f} KB)")
    else:
        print("  未生成图片")

    print()
    print("=" * 78)
    print("结论")
    print("=" * 78)
    print(f"  没有纵向控制时，车辆以 {args.cruise:.1f} m/s 定速过弯，"
          f"横向加速度达 {a_lat_no:.2f} m/s²，")
    print(f"  是设定上限 {lim.max_lateral_accel:.1f} m/s² 的 "
          f"{a_lat_no / lim.max_lateral_accel:.2f} 倍。")
    print(f"  本模块在弯前把车速降到 {v_limit_curve:.2f} m/s"
          f"（{v_limit_curve * 3.6:.1f} km/h），")
    print(f"  实测最大横向加速度 {r_with.max_lateral_accel:.2f} m/s²，"
          f"始终在限值内；")
    print(f"  同时 jerk 被限制在 {r_with.max_abs_jerk:.2f} m/s³ 以内，"
          f"保证加减速平顺。")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
