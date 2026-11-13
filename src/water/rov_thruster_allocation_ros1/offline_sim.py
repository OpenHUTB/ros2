#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROV 推力分配的离线验证与出图（不需要 ROS / 仿真器）。

用法（在仓库根目录执行）：
    python src/water/rov_thruster_allocation_ros1/offline_sim.py

内容：
1. 一段 12 s 的预设任务（前进、转向、下潜、组合运动），验证正常分配精度；
2. 饱和对比：期望推力超过单推进器上限时，clamp 与 scale 两种策略的差异；
3. 故障对比：T2 推进器失效后，用剩余推进器重新分配的效果。

输出：
    docs/img/water/rov_thruster_allocation_ros1/thruster_cmds.png
    docs/img/water/rov_thruster_allocation_ros1/achieved_vs_desired.png
    docs/img/water/rov_thruster_allocation_ros1/saturation_fault.png
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

from allocation import (  # noqa: E402
    THRUSTERS, WRENCH_NAMES, allocate, build_allocation_matrix, residual_ratio,
)

OUTPUT_DIR = os.path.join(REPO_ROOT, 'docs', 'img', 'water', 'rov_thruster_allocation_ros1')

RATE = 20.0
MAX_THRUST = 40.0
DURATION = 12.0


def mission_wrench(t):
    """12 s 预设任务：前进 -> 转向 -> 下潜抬头 -> 组合运动。"""
    if t < 3.0:
        return [30.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    if t < 6.0:
        return [0.0, 0.0, 0.0, 0.0, 0.0, 5.0]
    if t < 9.0:
        return [0.0, 0.0, 20.0, 0.0, 3.0, 0.0]
    return [20.0, 10.0, 12.0, 0.0, 0.0, 3.0]


def run_mission():
    steps = int(DURATION * RATE)
    times = np.arange(steps) / RATE
    commands = np.zeros((steps, len(THRUSTERS)))
    achieved = np.zeros((steps, 6))
    desired = np.zeros((steps, 6))
    ratios = np.zeros(steps)

    for k, t in enumerate(times):
        wrench = mission_wrench(t)
        result = allocate(wrench, max_thrust=MAX_THRUST, mode='scale')
        commands[k] = result['commands']
        achieved[k] = result['achieved']
        desired[k] = wrench
        ratios[k] = residual_ratio(result, wrench)

    return times, desired, achieved, commands, ratios


def wrench_angle_error_deg(wrench, result):
    """水平推力方向误差（度），用 Fx/Fy 两个分量计算。"""
    w = np.asarray(wrench[:2], dtype=float)
    a = np.asarray(result['achieved'][:2], dtype=float)
    nw, na = np.linalg.norm(w), np.linalg.norm(a)
    if nw < 1e-9 or na < 1e-9:
        return float('nan')
    cos_value = float(np.dot(w, a) / (nw * na))
    cos_value = max(-1.0, min(1.0, cos_value))
    return math.degrees(math.acos(cos_value))


def scenarios():
    """返回各场景的指标，用于柱状图。"""
    normal = allocate([20.0, 0.0, 0.0, 0.0, 0.0, 0.0], max_thrust=MAX_THRUST)
    over = [80.0, 10.0, 0.0, 0.0, 0.0, 10.0]   # 超过推进器能力
    clamp = allocate(over, max_thrust=MAX_THRUST, mode='clamp')
    scale = allocate(over, max_thrust=MAX_THRUST, mode='scale')
    fault_wrench = [30.0, 0.0, 0.0, 0.0, 0.0, 4.0]
    fault = allocate(fault_wrench, max_thrust=MAX_THRUST, failed=1)

    data = {
        'normal': residual_ratio(normal, [20.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        'clamp': residual_ratio(clamp, over),
        'scale': residual_ratio(scale, over),
        'fault': residual_ratio(fault, fault_wrench),
    }
    extra = {
        'normal_max_residual_pct': data['normal'],
        'saturation_clamp_residual_pct': data['clamp'],
        'saturation_scale_residual_pct': data['scale'],
        'saturation_scale_factor': scale['scale'],
        'saturation_clamp_angle_error_deg': wrench_angle_error_deg(over, clamp),
        'saturation_scale_angle_error_deg': wrench_angle_error_deg(over, scale),
        'fault_residual_pct': data['fault'],
        'fault_achieved': fault['achieved'].tolist(),
        'fault_commands': fault['commands'].tolist(),
        'matrix_rank': int(np.linalg.matrix_rank(build_allocation_matrix())),
    }
    return data, extra


def plot(times, desired, achieved, commands, ratios, bar_data):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # 1) 推进器指令
    styles = ['-', '--', '-.', ':']
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    for i in range(4):
        axes[0].plot(times, commands[:, i], label=THRUSTERS[i]['name'],
                     linewidth=1.6, linestyle=styles[i])
    axes[0].set_ylabel('thrust (N)')
    axes[0].set_title('Horizontal thruster commands during the mission')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc='upper right', fontsize=7, ncol=2)

    for i in (4, 5):
        axes[1].plot(times, commands[:, i], label=THRUSTERS[i]['name'],
                     linewidth=1.6, linestyle=styles[i - 4])
    axes[1].set_xlabel('time (s)')
    axes[1].set_ylabel('thrust (N)')
    axes[1].set_title('Vertical thruster commands')
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc='upper right', fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'thruster_cmds.png'), dpi=150)
    plt.close(fig)

    # 2) 期望 vs 实际 wrench（四个可控分量）
    components = [0, 1, 2, 5]   # Fx, Fy, Fz, Mz
    fig, axes = plt.subplots(2, 2, figsize=(9, 6), sharex=True)
    for ax, index in zip(axes.flat, components):
        ax.plot(times, desired[:, index], 'k--', linewidth=1.5, label='desired')
        ax.plot(times, achieved[:, index], color='#1f77b4', linewidth=1.5, label='achieved')
        ax.set_title(WRENCH_NAMES[index])
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=8)
    for ax in axes[1]:
        ax.set_xlabel('time (s)')
    fig.suptitle('Desired vs achieved wrench (normal operation)')
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'achieved_vs_desired.png'), dpi=150)
    plt.close(fig)

    # 3) 饱和与故障残差对比
    labels = ['normal', 'clamp\n(saturated)', 'scale\n(saturated)', 'fault T2']
    values = [bar_data['normal'], bar_data['clamp'], bar_data['scale'], bar_data['fault']]
    colors = ['#2ca02c', '#ff7f0e', '#1f77b4', '#d62728']
    fig, ax = plt.subplots(figsize=(7, 4.2))
    bars = ax.bar(labels, values, color=colors)
    for bar, value in zip(bars, values):
        ax.text(bar.get_x() + bar.get_width() / 2.0, value + 0.3,
                '%.2f%%' % value, ha='center', va='bottom', fontsize=9)
    ax.set_ylabel('relative wrench residual (%)')
    ax.set_title('Allocation residual: normal, saturation and fault')
    ax.grid(True, axis='y', alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'saturation_fault.png'), dpi=150)
    plt.close(fig)


def main():
    times, desired, achieved, commands, ratios = run_mission()
    bar_data, extra = scenarios()
    plot(times, desired, achieved, commands, ratios, bar_data)

    result = {
        'mission_max_residual_pct': float(ratios.max()),
        'mission_mean_residual_pct': float(ratios.mean()),
        'max_thruster_command_N': float(np.abs(commands).max()),
        'saturation_scale_factor': extra['saturation_scale_factor'],
        'saturation_clamp_residual_pct': extra['saturation_clamp_residual_pct'],
        'saturation_scale_residual_pct': extra['saturation_scale_residual_pct'],
        'saturation_clamp_angle_error_deg': extra['saturation_clamp_angle_error_deg'],
        'fault_T2_residual_pct': extra['fault_residual_pct'],
        'matrix_rank': extra['matrix_rank'],
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    for name in ('thruster_cmds.png', 'achieved_vs_desired.png', 'saturation_fault.png'):
        print('figure:', os.path.join(OUTPUT_DIR, name))
    return 0


if __name__ == '__main__':
    sys.exit(main())
