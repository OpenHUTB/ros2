#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
节点3: 可视化 (robot_visualizer)
订阅话题: /odom  (nav_msgs/Odometry)
作用   : 订阅里程计位姿, 用 matplotlib 画出一个"人形机器人"走路,
         保存为 GIF, 直观看到机器人确实在动。
运行(手动):  rosrun humanoid_base_sim robot_visualizer.py
产物   :  ~/.ros/humanoid_demo.gif
"""
import os
import math
import rospy
from nav_msgs.msg import Odometry

import matplotlib
matplotlib.use("Agg")                       # 无需弹窗, 直接出图
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class Humanoid:
    """2D 侧视小人: 头 + 躯干 + 两条会摆动的腿"""
    def __init__(self):
        self.head_r = 0.14
        self.body_h = 0.30
        self.leg_len = 0.42

    def draw(self, ax, x, y, th, phase):
        c, s = math.cos(th), math.sin(th)
        def P(lx, ly):
            return (x + lx * c - ly * s, y + lx * s + ly * c)

        hip = P(0.0, 0.0)
        shx, shy = P(0.0, self.body_h)
        hcx, hcy = P(0.0, self.body_h + self.head_r)

        swing = 0.75 * math.sin(phase)
        back = 0.75 * math.sin(phase + math.pi)

        ax.add_patch(plt.Circle((hcx, hcy), self.head_r, color="#f0c8a0", zorder=5))
        ax.plot([hip[0], shx], [hip[1], shy], color="#3b6db5",
                linewidth=6, solid_capstyle="round", zorder=4)
        fx1, fy1 = P(0.0, 0.0); fx2, fy2 = P(swing, -self.leg_len)
        ax.plot([fx1, fx2], [fy1, fy2], color="#3b6db5",
                linewidth=5, solid_capstyle="round", zorder=3)
        bx1, by1 = P(0.0, 0.0); bx2, by2 = P(back, -self.leg_len)
        ax.plot([bx1, bx2], [by1, by2], color="#2c4f7e",
                linewidth=5, solid_capstyle="round", zorder=3)


class Visualizer:
    def __init__(self):
        self.pose = (0.0, 0.0, 0.0)
        self.v = 0.0
        self.w = 0.0
        rospy.Subscriber("odom", Odometry, self._on_odom)

    def _on_odom(self, msg):
        qz, qw = msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        th = 2.0 * math.atan2(qz, qw)
        self.pose = (msg.pose.pose.position.x,
                     msg.pose.pose.position.y, th)
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z

    def run(self):
        rate = rospy.Rate(10)
        humanoid = Humanoid()
        trail = []
        phase = 0.0
        fig, ax = plt.subplots(figsize=(7, 6.4), dpi=110)

        def draw_axes():
            ax.set_xlim(-2.4, 2.4); ax.set_ylim(-2.4, 2.4)
            ax.set_aspect("equal"); ax.grid(True, linestyle="--", alpha=0.4)
            ax.set_title("ROS1 Noetic · 人形机器人基础仿真  (cmd_vel → odom)", fontsize=11)
            ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")

        frames = []
        while not rospy.is_shutdown():
            x, y, th = self.pose
            trail.append((x, y))
            phase += (1.0 / 10.0) * max(1.0, abs(self.v) * 6.0)

            ax.clear(); draw_axes()
            if len(trail) > 1:
                tx, ty = zip(*trail)
                ax.plot(tx, ty, color="#8fbfe0", linewidth=2, alpha=0.9, zorder=2)
            humanoid.draw(ax, x, y, th, phase)
            ax.text(0.02, 0.02,
                    f"cmd_vel: v={self.v:.2f} m/s   w={self.w:.2f} rad/s\n"
                    f"odom  : x={x:.2f} y={y:.2f} θ={math.degrees(th)%360:.0f}°",
                    transform=ax.transAxes, fontsize=9,
                    bbox=dict(boxstyle="round", fc="white", ec="#888", alpha=0.8))
            fig.canvas.draw()
            frame = np_from_canvas(fig)
            frames.append(frame)
            rate.sleep()

        gif = os.path.expanduser("~/.ros/humanoid_demo.gif")
        os.makedirs(os.path.dirname(gif), exist_ok=True)
        frames[0].save(gif, save_all=True, append_images=frames[1:],
                       duration=100, loop=0)
        rospy.loginfo("GIF 已保存: %s (%d 帧)", gif, len(frames))


def np_from_canvas(fig):
    """把当前 matplotlib 画布转成 PIL Image, 用于拼 GIF"""
    import numpy as np
    buf = fig.canvas.buffer_rgba()
    arr = np.asarray(buf).copy()
    from PIL import Image
    return Image.fromarray(arr).convert("RGB")


if __name__ == "__main__":
    rospy.init_node("robot_visualizer")
    Visualizer().run()
