#!/usr/bin/env python3
"""轨迹对比图：规划路径 vs 实际飞行轨迹"""
import numpy as np
import cv2
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE = os.path.expanduser('~/ros2/src/uav_slam_nav')
path_w = np.load(os.path.join(BASE, 'path.npy'))
nav = np.load(os.path.join(BASE, 'nav_track.npy'))

plt.figure(figsize=(8, 6))
plt.plot(path_w[:, 0], path_w[:, 1], 'b--', lw=1.5, label='Planned path (A*)')
plt.plot(nav[:, 0], nav[:, 1], 'r-o', markersize=3, lw=1.5, label='Actual flight')
plt.scatter(path_w[0, 0], path_w[0, 1], c='g', s=120, marker='o', label='Start')
plt.scatter(path_w[-1, 0], path_w[-1, 1], c='m', s=120, marker='x', label='Goal')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('AirSim SLAM Nav: Planned vs Actual')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.savefig(os.path.join(BASE, 'nav_track.png'), dpi=120, bbox_inches='tight')
print(f"对比图已保存: nav_track.png")
