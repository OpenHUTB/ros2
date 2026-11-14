#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot_viewer.py — 用 matplotlib 画小车轨迹 (ROS1 Noetic)

订阅:
  /odom  (nav_msgs/Odometry)

直接跑:
  rosrun ackermann_sim plot_viewer.py
"""

import math
import rospy
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrow
from nav_msgs.msg import Odometry


class PlotViewer:
    def __init__(self):
        rospy.init_node("ackermann_plot_viewer", anonymous=True)

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig.canvas.manager.set_window_title("Ackermann Viewer")
        self.ax.set_xlim(-15, 15)
        self.ax.set_ylim(-15, 15)
        self.ax.set_aspect("equal")
        self.ax.grid(True, alpha=0.3)

        self.traj_line, = self.ax.plot([], [], "b-", lw=1.2, label="trajectory")
        self.car_patch = None
        self.front_arrow = None
        self.tx, self.ty = [], []

        self.info = self.ax.text(0.02, 0.98, "", transform=self.ax.transAxes,
                                 va="top", fontsize=9, family="monospace")

        rospy.Subscriber("/odom", Odometry, self._odom_cb, queue_size=1)

    def _odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # 从四元数取 yaw
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = 2.0 * math.atan2(qz, qw)
        v = msg.twist.twist.linear.x

        self.tx.append(x)
        self.ty.append(y)
        self.traj_line.set_data(self.tx, self.ty)

        # 画车身
        L, W = 2.0, 1.0
        if self.car_patch is not None:
            self.car_patch.remove()
        self.car_patch = Rectangle((x - L/2, y - W/2), L, W,
                                   angle=math.degrees(theta),
                                   facecolor="#4C9AFF", edgecolor="black",
                                   linewidth=1.5, zorder=5)
        self.ax.add_patch(self.car_patch)

        self.info.set_text(f"x={x:6.2f}  y={y:6.2f}\ntheta={math.degrees(theta):6.1f}°  v={v:4.2f} m/s")

        # 跟随小车移动视野
        self.ax.set_xlim(x - 10, x + 10)
        self.ax.set_ylim(y - 10, y + 10)
        self.fig.canvas.draw_idle()

    def spin(self):
        plt.legend(loc="lower left")
        plt.show(block=True)


if __name__ == "__main__":
    try:
        PlotViewer().spin()
    except rospy.ROSInterruptException:
        pass
