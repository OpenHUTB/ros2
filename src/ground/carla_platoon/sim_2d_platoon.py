#!/usr/bin/env python3
"""
sim_2d_platoon.py —— 轻量 2D 编队仿真（matplotlib 渲染，可直接跑出效果动图）
================================================================================
与 CARLA 版共用同一套编队控制律（platoon_core.Platoon），因此本脚本跑出来的
动图即「多车协同编队」算法在轻量仿真中的真实运行效果：
  - Leader 沿参考轨迹匀速领航
  - 3 辆 follower 沿同一条轨迹保持恒定弧长间距跟进（一字纵队）
  - 转弯处队形不散

运行：
    python3 sim_2d_platoon.py                 # 生成 platoon_demo.gif
    python3 sim_2d_platoon.py --show          # 弹出窗口实时查看
    python3 sim_2d_platoon.py --frames 300 --fps 30 --out platoon_demo.gif

依赖：numpy, matplotlib, Pillow
"""
import argparse
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")          # 无显示环境也能出图
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation, PillowWriter

from platoon_core import (Platoon, build_path_from_waypoints)


# 参考轨迹：一段直线 + S 形横摆 + 直线（含连续转弯，验证队形保持）
def make_reference_path():
    xs = np.linspace(0.0, 160.0, 400)
    mid, w = 80.0, 12.0
    ys = np.where(
        xs < mid - 20, 0.0,
        np.where(xs > mid + 20, 0.0,
                 w * np.sin((xs - (mid - 20)) / 40.0 * np.pi))
    )
    # 采样成 waypoint 并稠密化
    wpts = np.column_stack([xs, ys])[::8]
    return build_path_from_waypoints(wpts, dense_per_m=2.0)


def draw_car(ax, state, color, label, size=(4.6, 1.8)):
    """按位姿画一辆车的矩形（阿克曼车横置，车头沿 yaw）。
    返回创建的 (rect, arrow, text)，供调用方逐帧统一清理，避免残影。"""
    x, y, yaw = state
    w, h = size
    rect = patches.Rectangle((-w / 2, -h / 2), w, h, linewidth=1.2,
                             edgecolor="black", facecolor=color, alpha=0.9)
    t = matplotlib.transforms.Affine2D().rotate(yaw).translate(x, y)
    rect.set_transform(t + ax.transData)
    ax.add_patch(rect)
    # 车头方向箭头
    arrow = ax.annotate("", xy=(x + 2.4 * np.cos(yaw), y + 2.4 * np.sin(yaw)),
                        xytext=(x, y), arrowprops=dict(arrowstyle="->", color="black",
                                                       lw=1.0, alpha=0.8))
    txt = ax.text(x, y - 2.2, label, ha="center", va="top", fontsize=10,
                  fontweight="bold", color=color)
    return rect, arrow, txt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=300)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--dt", type=float, default=0.033)
    ap.add_argument("--out", default="platoon_demo.gif")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    path, s = make_reference_path()
    platoon = Platoon(path, s, n_vehicles=4, spacing=8.0, v_set=6.0,
                      lookahead=6.0, L=2.8, kp=1.2)

    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]   # L, F1, F2, F3
    labels = ["L", "F1", "F2", "F3"]

    fig, ax = plt.subplots(figsize=(12, 5.5), dpi=100)
    ax.plot(path[:, 0], path[:, 1], "--", color="gray", lw=1.0, alpha=0.9,
            label="reference trajectory")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Multi-vehicle cooperative platoon (2D sim)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")
    # 尾迹
    trail = []
    trail_line, = ax.plot([], [], "-", lw=2.0, color="#1f77b4", alpha=0.5)
    info = ax.text(0.02, 0.95, "", transform=ax.transAxes, va="top",
                   fontsize=10, fontfamily="monospace",
                   bbox=dict(boxstyle="round", fc="white", ec="gray", alpha=0.8))
    prev_objs = []          # 统一维护每帧的车辆绘图对象，逐帧清理（防残影）

    def update(frame):
        states, targets, speeds = platoon.step(args.dt)
        for o in prev_objs:          # 清除上一帧的全部车辆图形（矩形/箭头/标签）
            o.remove()
        prev_objs.clear()
        for i in range(platoon.n):
            prev_objs.extend(draw_car(ax, states[i], colors[i], labels[i]))
        # leader 尾迹
        trail.append([states[0, 0], states[0, 1]])
        if len(trail) > 60:
            trail.pop(0)
        tr = np.asarray(trail)
        trail_line.set_data(tr[:, 0], tr[:, 1])
        # 队形信息
        gap_12 = np.linalg.norm(states[0, :2] - states[1, :2])
        gap_34 = np.linalg.norm(states[2, :2] - states[3, :2])
        info.set_text(
            f"t = {frame * args.dt:.1f} s   v = {speeds[0]:.1f} m/s\n"
            f"gap(L-F1) = {gap_12:.1f} m   gap(F2-F3) = {gap_34:.1f} m\n"
            f"spacing target = {platoon.spacing:.0f} m")
        return (trail_line, info)

    # 预跑一定帧数让队伍离开起点再开始录制
    for _ in range(30):
        platoon.step(args.dt)

    anim = FuncAnimation(fig, update, frames=args.frames, interval=1000 / args.fps,
                         blit=False)
    out_path = args.out
    writer = PillowWriter(fps=args.fps)
    anim.save(out_path, writer=writer, dpi=100)
    print(f"[OK] 动图已生成: {os.path.abspath(out_path)}  "
          f"({args.frames} frames @ {args.fps}fps, {os.path.getsize(out_path)//1024} KB)")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
