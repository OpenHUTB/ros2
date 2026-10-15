#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""节点3: 可视化 (cartpole_balance)
订阅 /cartpole/state, 实时绘制小车+摆杆, 并把最近一段状态历史保存为 GIF。

- 实时窗口用 matplotlib(Agg 无头模式)绘制小车(蓝矩形)与摆杆(红杆)
- 每收集约 6 秒数据保存一次动图到 ~/cartpole_balance.gif(可多次覆盖更新)
- 节点退出(Ctrl+C)时再保存最终动图

用法: roslaunch cartpole_balance main.launch
"""
import math
import os
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray

matplotlib_use = "Agg"  # 无显示器环境下绘图
import matplotlib
matplotlib.use(matplotlib_use)
import matplotlib.pyplot as plt
import matplotlib.animation as animation

GIF_PATH = os.path.expanduser("~/cartpole_balance.gif")   # 动图输出路径
WINDOW_SEC = 12.0     # 保存动图所用的时间窗口
SAVE_EVERY = 300      # 每 300 个采样(约6s)保存一次动图

hist = []             # 记录 [(x, theta), ...]


def save_gif():
    if len(hist) < 2:
        return
    xs = [h[0] for h in hist]
    ths = [h[1] for h in hist]

    fig, ax = plt.subplots(figsize=(8, 4.6), dpi=110)
    n = len(xs)
    every = max(1, n // 200)

    def draw_frame(i):
        x, th = xs[i], ths[i]
        ax.clear()
        ax.set_xlim(-2.5, 2.5)
        ax.set_ylim(-0.8, 2.4)
        ax.set_aspect("equal")
        ax.grid(True, linestyle="--", alpha=0.4)
        ax.set_title("倒立摆平衡 (cart-pole, LQR 最优控制 · ROS1 Noetic)", fontsize=11)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.plot([-2.5, 2.5], [0, 0], color="#444", linewidth=2)
        cart_w, cart_h = 0.7, 0.18
        ax.add_patch(plt.Rectangle((x - cart_w / 2, -cart_h), cart_w, cart_h,
                                   color="#3b6db5", zorder=3))
        pivot_x, pivot_y = x, 0.0
        tip_x = pivot_x + 2 * 0.5 * math.sin(th)
        tip_y = pivot_y + 2 * 0.5 * math.cos(th)
        ax.plot([pivot_x, tip_x], [pivot_y, tip_y], color="#c0392b",
                linewidth=5, solid_capstyle="round", zorder=4)
        ax.plot(pivot_x, pivot_y, "o", color="#222", markersize=6, zorder=5)
        ax.plot(tip_x, tip_y, "o", color="#c0392b", markersize=7, zorder=5)
        ax.text(0.02, 0.95,
                f"θ = {math.degrees(th):+.1f}°   x = {x:+.2f} m",
                transform=ax.transAxes, fontsize=10,
                bbox=dict(boxstyle="round", fc="white", ec="#888", alpha=0.85))
        return []

    try:
        frames = len(range(0, n, every))
        fps = max(1, int(frames / WINDOW_SEC))   # 让 GIF 时长≈WINDOW_SEC
        anim = animation.FuncAnimation(fig, draw_frame, frames=frames, blit=False)
        anim.save(GIF_PATH, writer="pillow", fps=fps, dpi=110)
        rospy.loginfo("已保存动图: %s (%d 帧)", GIF_PATH, frames)
    except Exception as e:
        rospy.logwarn("保存动图失败: %s", e)
    plt.close(fig)


def on_state(msg):
    global hist
    x, th = msg.data[0], msg.data[2]
    hist.append((x, th))
    # 只保留最近 WINDOW_SEC 秒的采样(约 50Hz)
    max_hist = int(50 * WINDOW_SEC)
    if len(hist) > max_hist:
        hist = hist[-max_hist:]
    rospy.loginfo_throttle(5.0, "x=%.3f m, theta=%.1f deg", x, math.degrees(th))
    if len(hist) % SAVE_EVERY == 0:
        save_gif()


if __name__ == "__main__":
    rospy.init_node("cartpole_visualizer")
    rospy.Subscriber("/cartpole/state", Float64MultiArray, on_state)
    rospy.loginfo("cartpole_visualizer 已启动: 订阅 /cartpole/state, 动图输出 %s", GIF_PATH)
    rospy.on_shutdown(save_gif)
    rospy.spin()
