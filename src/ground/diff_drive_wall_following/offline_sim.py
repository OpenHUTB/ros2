#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""差速小车沿墙行驶的离线仿真与出图（不需要 ROS / Gazebo）。

用法（在仓库根目录执行）：
    python src/ground/diff_drive_wall_following/offline_sim.py

仿真内容：
- 一个 6 m x 5 m 的矩形房间，小车从南墙附近出发，沿右侧墙壁逆时针绕行一圈；
- 按真实激光雷达的角度分辨率（1°）做射线求交，取五路最小距离；
- 差速小车运动学积分 + 执行器一阶滞后，模拟速度指令到实际速度的延迟。

输出：
    docs/img/ground/diff_drive_wall_following/map_trajectory.png
    docs/img/ground/diff_drive_wall_following/wall_distance.png
    docs/img/ground/diff_drive_wall_following/velocity.png
"""
from __future__ import print_function

import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, HERE)

from wall_follower import STATE_FOLLOW, STATE_TURN, WallFollower  # noqa: E402

OUTPUT_DIR = os.path.join(REPO_ROOT, 'docs', 'img', 'ground', 'diff_drive_wall_following')

ROOM_WIDTH = 6.0
ROOM_HEIGHT = 5.0
START = (1.2, 0.5, 0.0)     # x, y, heading
DT = 0.05
TAU = 0.15                  # 执行器时间常数
MAX_TIME = 120.0
BEAM_RESOLUTION = math.radians(1.0)
MAX_RANGE = 8.0

BEAM_CENTERS = {
    'front': 0.0,
    'side': -math.pi / 2,
    'front_right': -math.pi / 4,
    'back_right': -3 * math.pi / 4,
    'left': math.pi / 2,
}
WINDOW = math.radians(12)


def ray_distance(x, y, angle):
    """从 (x, y) 沿 angle 发射射线，返回到矩形房间四壁的最近距离。"""
    dx, dy = math.cos(angle), math.sin(angle)
    best = MAX_RANGE
    if dx > 1e-9:
        best = min(best, (ROOM_WIDTH - x) / dx)
    elif dx < -1e-9:
        best = min(best, (0.0 - x) / dx)
    if dy > 1e-9:
        best = min(best, (ROOM_HEIGHT - y) / dy)
    elif dy < -1e-9:
        best = min(best, (0.0 - y) / dy)
    return max(0.0, best)


def scan(x, y, heading, noise_std=0.0, rng=None):
    """模拟一圈 1° 激光，返回五路最小距离。"""
    ranges = {}
    for name, center in BEAM_CENTERS.items():
        best = MAX_RANGE
        n = int(round(WINDOW / BEAM_RESOLUTION))
        for k in range(-n, n + 1):
            a = heading + center + k * BEAM_RESOLUTION
            d = ray_distance(x, y, a)
            if noise_std > 0 and rng is not None:
                d += rng.normal(0.0, noise_std)
            if d < best:
                best = d
        ranges[name] = best
    return ranges


def simulate(noise_std=0.01, seed=7, follower_kwargs=None):
    rng = np.random.RandomState(seed)
    follower = WallFollower(**(follower_kwargs or {}))

    x, y, heading = START
    v = 0.0
    w = 0.0

    times = [0.0]
    xs = [x]
    ys = [y]
    headings = [heading]
    vs = [v]
    ws = [w]
    sides = [np.nan]
    fronts = [np.nan]
    states = [follower.state]

    t = 0.0
    lap_time = None
    traveled = 0.0
    while t < MAX_TIME:
        ranges = scan(x, y, heading, noise_std=noise_std, rng=rng)
        cmd_v, cmd_w, state = follower.update(
            ranges['front'], ranges['side'], ranges['front_right'],
            ranges['back_right'], ranges['left'], DT)

        # 执行器一阶滞后
        alpha = min(DT / TAU, 1.0)
        v += (cmd_v - v) * alpha
        w += (cmd_w - w) * alpha

        x += v * math.cos(heading) * DT
        y += v * math.sin(heading) * DT
        heading += w * DT

        traveled += abs(v) * DT
        t += DT

        times.append(t)
        xs.append(x)
        ys.append(y)
        headings.append(heading)
        vs.append(v)
        ws.append(w)
        sides.append(ranges['side'])
        fronts.append(ranges['front'])
        states.append(state)

        if lap_time is None and t > 10.0 and traveled > 14.0:
            heading_error = abs(math.atan2(math.sin(heading - START[2]),
                                           math.cos(heading - START[2])))
            if x > 1.0 and y < 1.2 and heading_error < 0.3:
                lap_time = t
                break

    return {
        't': np.array(times), 'x': np.array(xs), 'y': np.array(ys),
        'heading': np.array(headings), 'v': np.array(vs), 'w': np.array(ws),
        'side': np.array(sides), 'front': np.array(fronts),
        'state': np.array(states), 'lap_time': lap_time, 'traveled': traveled,
    }


def metrics(data):
    t = data['t']
    side = data['side']
    states = data['state']
    follow = states == STATE_FOLLOW
    err = np.abs(side[follow] - 0.5)
    err = err[~np.isnan(err)]
    return {
        'lap_time_s': data['lap_time'],
        'path_length_m': float(data['traveled']),
        'mean_wall_error_m': float(err.mean()) if len(err) else float('nan'),
        'max_wall_error_m': float(err.max()) if len(err) else float('nan'),
        'min_front_clearance_m': float(np.nanmin(data['front'])),
        'final_position_m': [float(data['x'][-1]), float(data['y'][-1])],
    }


def plot(data):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # 1) 地图与轨迹
    fig, ax = plt.subplots(figsize=(7, 5.5))
    ax.add_patch(Rectangle((0, 0), ROOM_WIDTH, ROOM_HEIGHT, fill=False,
                           edgecolor='black', linewidth=2))
    colors = {STATE_FOLLOW: ('FOLLOW', '#1f77b4'),
              STATE_TURN: ('TURN', '#d62728')}
    for state, (label, color) in colors.items():
        mask = data['state'] == state
        if mask.any():
            ax.plot(data['x'][mask], data['y'][mask], color=color, linewidth=2,
                    label=label)
    ax.plot(START[0], START[1], 'go', markersize=8, label='start')
    ax.plot(data['x'][-1], data['y'][-1], 'rx', markersize=10, label='end')
    ax.set_xlim(-0.4, ROOM_WIDTH + 0.4)
    ax.set_ylim(-0.4, ROOM_HEIGHT + 0.4)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Wall-following trajectory (right-hand rule)')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'map_trajectory.png'), dpi=150)
    plt.close(fig)

    # 2) 右侧墙距离
    fig, ax = plt.subplots(figsize=(8, 4.2))
    ax.plot(data['t'], data['side'], color='#1f77b4', linewidth=1.5, label='right wall distance')
    ax.axhline(0.5, color='gray', linestyle='--', linewidth=1, label='target 0.5 m')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('distance (m)')
    ax.set_title('Distance to the right wall')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right')
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'wall_distance.png'), dpi=150)
    plt.close(fig)

    # 3) 速度
    fig, ax = plt.subplots(figsize=(8, 4.2))
    ax.plot(data['t'], data['v'], label='linear velocity v (m/s)')
    ax.plot(data['t'], data['w'], label='angular velocity w (rad/s)')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('command')
    ax.set_title('Velocity commands while following the wall')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right')
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'velocity.png'), dpi=150)
    plt.close(fig)


def main():
    data = simulate()
    result = metrics(data)
    plot(data)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    for name in ('map_trajectory.png', 'wall_distance.png', 'velocity.png'):
        print('figure:', os.path.join(OUTPUT_DIR, name))
    return 0


if __name__ == '__main__':
    sys.exit(main())
