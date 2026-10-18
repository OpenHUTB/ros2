#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
人形机器人轨迹可视化 (matplotlib)
订阅 /odom 画机器人位置和轨迹
"""
import math
import rospy
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrow
from nav_msgs.msg import Odometry


class HumanoidViewer:
    def __init__(self):
        rospy.init_node("humanoid_viewer", anonymous=True)
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig.canvas.manager.set_window_title("Humanoid Viewer")
        self.ax.set_aspect("equal")
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.traj_line, = self.ax.plot([], [], "g-", lw=1.5, label="trajectory")
        self.robot_circle = None
        self.robot_arrow = None
        self.tx, self.ty = [], []
        self.info = self.ax.text(0.02, 0.98, "", transform=self.ax.transAxes,
                                 va="top", fontsize=9, family="monospace")
        rospy.Subscriber("/odom", Odometry, self._cb)

    def _cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = 2.0 * math.atan2(qz, qw)
        v = msg.twist.twist.linear.x
        self.tx.append(x)
        self.ty.append(y)
        self.traj_line.set_data(self.tx, self.ty)
        if self.robot_circle is not None:
            self.robot_circle.remove()
        if self.robot_arrow is not None:
            self.robot_arrow.remove()
        self.robot_circle = Circle((x, y), 0.3, facecolor="#FF6B6B",
                                   edgecolor="black", linewidth=1.5, zorder=5)
        self.ax.add_patch(self.robot_circle)
        self.robot_arrow = FancyArrow(x, y, 0.6 * math.cos(theta),
                                      0.6 * math.sin(theta),
                                      width=0.05, color="blue", zorder=6)
        self.ax.add_patch(self.robot_arrow)
        self.info.set_text(
            f"x={x:6.2f}  y={y:6.2f}\n"
            f"theta={math.degrees(theta):6.1f}  v={v:4.2f} m/s\n"
            f"[W]forward [S]back [A]left [D]right")
        self.ax.set_xlim(x - 8, x + 8)
        self.ax.set_ylim(y - 8, y + 8)
        self.fig.canvas.draw_idle()

    def spin(self):
        plt.legend(loc="lower left")
        plt.show()


if __name__ == "__main__":
    try:
        HumanoidViewer().spin()
    except rospy.ROSInterruptException:
        pass
