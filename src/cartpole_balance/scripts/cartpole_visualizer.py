#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""节点3: 可视化 (cartpole_balance)
订阅 /cartpole/state, 实时绘制小车+摆杆, 并把最近一段状态历史异步保存为 GIF。

- 绘图用 matplotlib(Agg 无头模式): 小车(蓝矩形) + 摆杆(红杆)
- 每 6 秒(300 个 50Hz 采样)把最近 12 秒的状态窗口另存为动图
  到 ~/cartpole_balance.gif; 保存放在后台守护线程, 不阻塞 50Hz 状态回调,
  避免 25s 的 GIF 编码期间状态消息堆积、曲线断档
- 节点退出(Ctrl+C)时在主线程再同步保存一次最终动图

中文字体: Ubuntu 20.04 默认不带中文字体, GIF 里的中文会渲染成方框。
节点启动时探测系统中文字体, 有则用中文标题, 无则自动退回英文标题。

用法: roslaunch cartpole_balance main.launch
"""
import math
import os
import threading

import rospy
from std_msgs.msg import Float64MultiArray

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402  (must follow matplotlib.use)
import matplotlib.animation as animation  # noqa: E402
from matplotlib import font_manager  # noqa: E402

GIF_PATH = os.path.expanduser("~/cartpole_balance.gif")   # 动图输出路径
PUB_HZ = 50.0        # 状态发布频率 Hz(与 cartpole_sim 一致)
WINDOW_SEC = 12.0    # 每张 GIF 覆盖的时间窗口长度
SAVE_EVERY = 300     # 每 300 个采样(=300/PUB_HZ=6s)触发一次后台保存

# 按优先级尝试的常见中文字体; Ubuntu 20.04 装 fonts-wqy-zenhei / fonts-noto-cjk 即可
_CJK_FONTS = ["Noto Sans CJK SC", "WenQuanYi Zen Hei", "WenQuanYi Micro Hei",
              "SimHei", "Microsoft YaHei", "Arial Unicode MS"]

_hist_lock = threading.Lock()
_hist = []            # [(x, theta), ...] 仅在 _hist_lock 内读写
_saving = False       # 后台是否正在编码 GIF(防重入)
_TITLE = None         # 在 main() 里按中文字体探测结果选定


def _pick_title():
    """探测系统是否有可用中文字体; 没有就退回英文标题, 避免中文渲染成方框。"""
    available = {f.name for f in font_manager.fontManager.ttflist}
    for name in _CJK_FONTS:
        if name in available:
            plt.rcParams["font.sans-serif"] = [name] + plt.rcParams["font.sans-serif"]
            plt.rcParams["axes.unicode_minus"] = False
            return "倒立摆平衡 (cart-pole, LQR 最优控制 · ROS1 Noetic)"
    rospy.logwarn("未检测到中文字体(可 sudo apt install fonts-wqy-zenhei), "
                  "GIF 标题改用英文, 避免中文渲染成方框")
    return "Cart-Pole Balancing (LQR optimal control · ROS1 Noetic)"


def _render_gif(points, title):
    """在调用线程内把 points=[(x,theta),...] 渲染成 GIF。不要在 rospy 回调里直接调。"""
    xs = [p[0] for p in points]
    ths = [p[1] for p in points]
    n = len(xs)
    every = max(1, n // 200)

    fig, ax = plt.subplots(figsize=(8, 4.6), dpi=110)

    def draw_frame(i):
        x, th = xs[i], ths[i]
        ax.clear()
        ax.set_xlim(-2.5, 2.5)
        ax.set_ylim(-0.8, 2.4)
        ax.set_aspect("equal")
        ax.grid(True, linestyle="--", alpha=0.4)
        ax.set_title(title, fontsize=11)
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
    finally:
        plt.close(fig)


def _save_gif_async():
    """复制一份历史快照后, 用后台守护线程编码 GIF, 立即返回不阻塞回调。"""
    global _saving
    with _hist_lock:
        if _saving or len(_hist) < 2:
            return
        snapshot = list(_hist)
        _saving = True
    title = _TITLE

    def worker():
        global _saving
        try:
            _render_gif(snapshot, title)
        finally:
            with _hist_lock:
                _saving = False

    threading.Thread(target=worker, name="gif-saver", daemon=True).start()


def _save_gif_now():
    """节点退出时在主线程同步保存一次(此时已无 50Hz 回调需要服务)。"""
    with _hist_lock:
        snapshot = list(_hist)
    if len(snapshot) >= 2 and _TITLE is not None:
        _render_gif(snapshot, _TITLE)


def on_state(msg):
    """状态回调: 校验消息长度, 畸形消息跳过; 只追加历史, 保存交给后台线程。"""
    try:
        data = msg.data
        if data is None or len(data) < 3:
            rospy.logwarn_throttle(
                5.0, "状态消息长度不足(len=%s, 至少需要 3), 已跳过",
                None if data is None else len(data))
            return
        x, th = float(data[0]), float(data[2])
    except (ValueError, TypeError, IndexError) as e:
        rospy.logwarn_throttle(5.0, "状态消息解析失败(%s), 已跳过", e)
        return

    max_hist = int(PUB_HZ * WINDOW_SEC)
    with _hist_lock:
        _hist.append((x, th))
        if len(_hist) > max_hist:
            del _hist[:-max_hist]
        n = len(_hist)

    rospy.loginfo_throttle(5.0, "x=%.3f m, theta=%.1f deg", x, math.degrees(th))
    if n % SAVE_EVERY == 0:
        _save_gif_async()


def main():
    global _TITLE
    rospy.init_node("cartpole_visualizer")
    _TITLE = _pick_title()
    rospy.Subscriber("/cartpole/state", Float64MultiArray, on_state)
    rospy.loginfo("cartpole_visualizer 已启动: 订阅 /cartpole/state, 动图输出 %s", GIF_PATH)
    rospy.on_shutdown(_save_gif_now)
    rospy.spin()


if __name__ == "__main__":
    main()
