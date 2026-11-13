#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""倒立摆能量摆起 + LQR 平衡的离线仿真与出图（不需要 ROS / Gazebo）。

用法（在仓库根目录执行）：
    python src/classical_control/cartpole_swingup/offline_sim.py

仿真内容：
1. 摆杆从自然下垂附近出发，能量控制器把能量泵到竖直向上的目标能量，
   接近竖直时切换到 LQR 保持平衡；
2. 对照组：只有 LQR（按竖直平衡点线性化）从下垂位置出发，验证它无法摆起；
3. 输出角度、能量、小车位置、控制力曲线与相图。

输出：
    docs/img/classical_control/cartpole_swingup/swingup_response.png
    docs/img/classical_control/cartpole_swingup/phase_portrait.png
    docs/img/classical_control/cartpole_swingup/lqr_only_comparison.png
"""
from __future__ import print_function

import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, HERE)

from cartpole_model import (  # noqa: E402
    MODE_BALANCE, MODE_SWINGUP, CartPoleController, cartpole_dynamics,
    total_energy, upright_energy, wrap_angle,
)

OUTPUT_DIR = os.path.join(REPO_ROOT, 'docs', 'img', 'classical_control', 'cartpole_swingup')

DT = 0.002          # 积分步长
CONTROL_DT = 0.02   # 控制周期（50 Hz）
DURATION = 20.0
INITIAL_STATE = [0.0, 0.0, math.pi - 0.1, 0.0]


def rk4_step(state, force, dt):
    k1 = cartpole_dynamics(state, force)
    k2 = cartpole_dynamics([s + 0.5 * dt * d for s, d in zip(state, k1)], force)
    k3 = cartpole_dynamics([s + 0.5 * dt * d for s, d in zip(state, k2)], force)
    k4 = cartpole_dynamics([s + dt * d for s, d in zip(state, k3)], force)
    return [s + dt / 6.0 * (a + 2 * b + 2 * c + d)
            for s, a, b, c, d in zip(state, k1, k2, k3, k4)]


def simulate(use_swingup=True, duration=DURATION):
    controller = CartPoleController(
        switch_angle=0.4, switch_rate=2.0, fall_angle=0.7,
        allow_swingup=use_swingup)

    state = list(INITIAL_STATE)
    t = 0.0
    control_timer = 0.0
    force = 0.0

    times = [t]
    states = [list(state)]
    forces = [force]
    modes = [controller.mode]

    switch_time = None
    settle_time = None
    theta_settle_time = None

    while t < duration:
        if control_timer <= 1e-12:
            force = controller.update(state, CONTROL_DT)
            control_timer += CONTROL_DT
        state = rk4_step(state, force, DT)
        t += DT
        control_timer -= DT

        times.append(t)
        states.append(list(state))
        forces.append(force)
        modes.append(controller.mode)

        if use_swingup and switch_time is None and controller.mode == MODE_BALANCE:
            switch_time = t
        if (use_swingup and switch_time is not None and theta_settle_time is None
                and abs(wrap_angle(state[2])) < 0.05 and abs(state[3]) < 0.2):
            theta_settle_time = t
        if (use_swingup and switch_time is not None and settle_time is None
                and abs(wrap_angle(state[2])) < 0.05 and abs(state[3]) < 0.2
                and abs(state[1]) < 0.1 and abs(state[0]) < 0.1):
            settle_time = t

    return {
        't': np.array(times),
        'state': np.array(states),
        'force': np.array(forces),
        'mode': np.array(modes),
        'switch_time': switch_time,
        'theta_settle_time': theta_settle_time,
        'settle_time': settle_time,
    }


def metrics(data):
    state = data['state']
    theta = np.array([wrap_angle(v) for v in state[:, 2]])
    energy = np.array([total_energy(s) for s in state])
    forces = data['force']
    return {
        'swing_up_time_s': data['switch_time'],
        'theta_settle_time_s': data['theta_settle_time'],
        'settle_time_s': data['settle_time'],
        'final_angle_deg': float(math.degrees(theta[-1])),
        'final_cart_position_m': float(state[-1, 0]),
        'max_cart_travel_m': float(np.max(np.abs(state[:, 0]))),
        'max_force_N': float(np.max(np.abs(forces))),
        'energy_target_J': float(upright_energy()),
        'final_energy_J': float(energy[-1]),
    }


def plot_swingup(data):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    t = data['t']
    state = data['state']
    theta = np.array([wrap_angle(v) for v in state[:, 2]])
    energy = np.array([total_energy(s) for s in state])
    force = data['force']
    balance = data['mode'] == MODE_BALANCE

    fig, axes = plt.subplots(2, 2, figsize=(10, 6.5), sharex=True)
    panels = [
        (axes[0, 0], theta, 'theta (rad)', 'Pendulum angle (0 = upright)'),
        (axes[0, 1], energy, 'energy (J)', 'Total mechanical energy'),
        (axes[1, 0], state[:, 0], 'x (m)', 'Cart position'),
        (axes[1, 1], force, 'force (N)', 'Control force'),
    ]
    for ax, values, ylabel, title in panels:
        ax.plot(t, values, color='#1f77b4', linewidth=1.2)
        ax.axvspan(t[balance][0] if balance.any() else t[-1], t[-1],
                   color='#2ca02c', alpha=0.12)
        ax.set_ylabel(ylabel)
        ax.set_title(title, fontsize=9)
        ax.grid(True, alpha=0.3)
    axes[0, 1].axhline(upright_energy(), color='gray', linestyle='--', linewidth=1)
    axes[1, 0].axhline(0.0, color='gray', linestyle='--', linewidth=0.8)
    axes[1, 1].axhline(0.0, color='gray', linestyle='--', linewidth=0.8)
    for ax in axes[1]:
        ax.set_xlabel('time (s)')
    fig.suptitle('Energy-based swing-up and LQR balance (green: balance mode)')
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'swingup_response.png'), dpi=150)
    plt.close(fig)


def plot_phase(data):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    theta = np.array([wrap_angle(v) for v in data['state'][:, 2]])
    rate = data['state'][:, 3]
    balance = data['mode'] == MODE_BALANCE

    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(theta[~balance], rate[~balance], color='#ff7f0e', linewidth=1.0,
            label='swing-up')
    ax.plot(theta[balance], rate[balance], color='#2ca02c', linewidth=1.4,
            label='LQR balance')
    ax.scatter([theta[0]], [rate[0]], color='black', s=30, label='start')
    ax.scatter([theta[-1]], [rate[-1]], color='red', marker='x', s=50, label='end')
    ax.axvline(0.0, color='gray', linestyle='--', linewidth=0.8)
    ax.set_xlabel('theta (rad)')
    ax.set_ylabel('theta_dot (rad/s)')
    ax.set_title('Phase portrait: swinging up to the upright equilibrium')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'phase_portrait.png'), dpi=150)
    plt.close(fig)


def plot_comparison(swingup, lqr_only):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    def cos_theta(data):
        return np.cos(np.array([wrap_angle(v) for v in data['state'][:, 2]]))

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(swingup['t'], cos_theta(swingup), label='energy swing-up + LQR')
    ax.plot(lqr_only['t'], cos_theta(lqr_only), '--', label='LQR only (from hanging)')
    ax.axhline(1.0, color='gray', linestyle=':', linewidth=0.8)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('cos(theta)   (1 = upright, -1 = hanging)')
    ax.set_title('Why swing-up is needed: LQR alone never reaches the upright position')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='lower right', fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, 'lqr_only_comparison.png'), dpi=150)
    plt.close(fig)


def main():
    swingup = simulate(use_swingup=True)
    lqr_only = simulate(use_swingup=False)

    plot_swingup(swingup)
    plot_phase(swingup)
    plot_comparison(swingup, lqr_only)

    result = metrics(swingup)
    result['lqr_only_final_angle_deg'] = float(
        math.degrees(wrap_angle(lqr_only['state'][-1, 2])))
    print(json.dumps(result, ensure_ascii=False, indent=2))
    for name in ('swingup_response.png', 'phase_portrait.png', 'lqr_only_comparison.png'):
        print('figure:', os.path.join(OUTPUT_DIR, name))
    return 0


if __name__ == '__main__':
    sys.exit(main())
