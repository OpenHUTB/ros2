#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""起飞—悬停—降落的离线仿真与出图（不需要 ROS / 飞控 / 仿真器）。

用法（在仓库根目录执行）：
    python src/air/uav_takeoff_hover_land/offline_sim.py

仿真模型：把状态机输出的速度指令当作期望速度，经过一阶惯性环节
（时间常数 tau）得到实际速度，再积分得到位置。它只验证状态机与
比例控制律的逻辑，不替代 PX4/Gazebo 等真实动力学仿真。

输出：
    docs/img/air/uav_takeoff_hover_land/trajectory_3d.png
    docs/img/air/uav_takeoff_hover_land/altitude_state.png
    docs/img/air/uav_takeoff_hover_land/velocity.png
并在终端打印起飞时间、悬停误差、降落时间等指标。
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

from flight_state_machine import (  # noqa: E402
    STATE_DONE, STATE_HOVER, STATE_LAND, STATE_TAKEOFF,
    TakeoffHoverLandFSM,
)

OUTPUT_DIR = os.path.join(REPO_ROOT, 'docs', 'img', 'air', 'uav_takeoff_hover_land')

TAKEOFF_ALTITUDE = 2.0
HOVER_TIME = 5.0
HOVER_OFFSET = (1.0, 0.5)   # 悬停点相对起飞点的水平偏移（m）
DT = 0.02
TAU = 0.25          # 速度响应时间常数（s）
MAX_TIME = 60.0


def simulate():
    """运行离线仿真，返回 (时间, 位置, 实际速度, 状态, 悬停目标)。"""
    fsm = TakeoffHoverLandFSM(
        takeoff_altitude=TAKEOFF_ALTITUDE,
        hover_time=HOVER_TIME,
        hover_offset=HOVER_OFFSET,
    )

    position = np.zeros(3)
    velocity = np.zeros(3)

    times = [0.0]
    positions = [position.copy()]
    velocities = [velocity.copy()]
    states = [fsm.state]

    t = 0.0
    hover_target = None
    while t < MAX_TIME:
        command = fsm.update(position, DT)
        if fsm.state == STATE_HOVER and hover_target is None:
            hover_target = fsm.hover_target

        target_v = np.array([command.vx, command.vy, command.vz])
        # 一阶惯性：实际速度跟踪指令速度
        alpha = min(DT / TAU, 1.0)
        velocity = velocity + (target_v - velocity) * alpha
        position = position + velocity * DT
        if position[2] < 0.0:
            position[2] = 0.0
            velocity[2] = 0.0

        t += DT
        times.append(t)
        positions.append(position.copy())
        velocities.append(velocity.copy())
        states.append(fsm.state)

        if command.state == STATE_DONE and t > 0:
            break

    return (np.array(times), np.array(positions), np.array(velocities),
            np.array(states), np.array(hover_target if hover_target else [0, 0, TAKEOFF_ALTITUDE]))


def compute_metrics(times, positions, states, hover_target):
    def first_time(state):
        idx = np.where(states == state)[0]
        return float(times[idx[0]]) if len(idx) else None

    def last_time(state):
        idx = np.where(states == state)[0]
        return float(times[idx[-1]]) if len(idx) else None

    t_takeoff_end = first_time(STATE_HOVER)
    t_land_start = first_time(STATE_LAND)
    t_land_end = first_time(STATE_DONE)

    hover_mask = states == STATE_HOVER
    if hover_mask.any():
        err = positions[hover_mask] - hover_target
        hover_rmse = float(np.sqrt(np.mean(np.sum(err * err, axis=1))))
    else:
        hover_rmse = float('nan')

    max_altitude = float(positions[:, 2].max())
    overshoot = max_altitude - TAKEOFF_ALTITUDE
    final_position = positions[-1].tolist()

    return {
        'takeoff_time_s': t_takeoff_end,
        'hover_time_measured_s': (t_land_start - t_takeoff_end) if (t_land_start and t_takeoff_end) else None,
        'hover_rmse_m': hover_rmse,
        'max_altitude_m': max_altitude,
        'altitude_overshoot_m': overshoot,
        'landing_time_s': (t_land_end - t_land_start) if (t_land_end and t_land_start) else None,
        'total_time_s': float(times[-1]),
        'final_position_m': final_position,
    }


def plot(times, positions, velocities, states, hover_target, metrics):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    # matplotlib 3.1.x 需要显式导入 Axes3D 才会注册 '3d' 投影，
    # 否则 add_subplot(projection='3d') 会抛 Unknown projection '3d'。
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    state_colors = {
        STATE_TAKEOFF: '#1f77b4',
        STATE_HOVER: '#2ca02c',
        STATE_LAND: '#d62728',
        STATE_DONE: '#7f7f7f',
    }
    state_labels = {
        STATE_TAKEOFF: 'takeoff',
        STATE_HOVER: 'hover',
        STATE_LAND: 'land',
        STATE_DONE: 'done',
    }

    # 1) 三维轨迹
    fig = plt.figure(figsize=(7, 5.5))
    ax = fig.add_subplot(111, projection='3d')
    for state, color in state_colors.items():
        mask = states == state
        if mask.any():
            ax.plot(positions[mask, 0], positions[mask, 1], positions[mask, 2],
                    color=color, linewidth=2, label=state_labels[state])
    ax.scatter(*positions[0], color='black', marker='o', s=30)
    ax.scatter(*hover_target, color='green', marker='*', s=120)
    ax.scatter(*positions[-1], color='red', marker='x', s=40)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_title('Takeoff - hover - land trajectory')
    ax.legend(loc='upper right')
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'trajectory_3d.png'), dpi=150)
    plt.close(fig)

    # 2) 高度与状态
    fig, ax = plt.subplots(figsize=(8, 4.2))
    for state, color in state_colors.items():
        mask = states == state
        if mask.any():
            ax.plot(times[mask], positions[mask, 2], color=color, linewidth=2,
                    label=state_labels[state])
    ax.axhline(TAKEOFF_ALTITUDE, color='gray', linestyle='--', linewidth=1,
               label='target altitude')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('z (m)')
    ax.set_title('Altitude and flight state')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='lower right')
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'altitude_state.png'), dpi=150)
    plt.close(fig)

    # 3) 速度曲线
    fig, ax = plt.subplots(figsize=(8, 4.2))
    ax.plot(times, velocities[:, 0], label='vx (forward)')
    ax.plot(times, velocities[:, 1], label='vy (left)')
    ax.plot(times, velocities[:, 2], label='vz (up)')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('velocity (m/s)')
    ax.set_title('Velocity commands tracking')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right')
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'velocity.png'), dpi=150)
    plt.close(fig)

    return [os.path.join(OUTPUT_DIR, name) for name in
            ('trajectory_3d.png', 'altitude_state.png', 'velocity.png')]


def main():
    times, positions, velocities, states, hover_target = simulate()
    metrics = compute_metrics(times, positions, states, hover_target)
    figures = plot(times, positions, velocities, states, hover_target, metrics)

    print(json.dumps(metrics, ensure_ascii=False, indent=2))
    for path in figures:
        print('figure:', path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
