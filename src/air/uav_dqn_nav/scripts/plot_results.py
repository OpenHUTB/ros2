#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""画 DQN 训练学习曲线 + 飞行轨迹图（实验结果可视化）。

用法：在 uav_dqn_nav 目录内执行 `python3 scripts/plot_results.py`，
输出 `models/training_curve.png`（训练曲线）与 `models/trajectory.png`（飞行轨迹）。
依赖：numpy + matplotlib（如遇中文字体问题，在 Windows 上会自动加载微软雅黑）。
"""
import os
import re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as mfm

# 中文字体：Windows 直接注册微软雅黑；其他平台退化为系统默认
for cand in [r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\msyh.ttf",
             r"C:\Windows\Fonts\simhei.ttf", r"C:\Windows\Fonts\simsun.ttc"]:
    if os.path.exists(cand):
        mfm.fontManager.addfont(cand)
        plt.rcParams["font.sans-serif"] = [mfm.FontProperties(fname=cand).get_name()]
        break
plt.rcParams["axes.unicode_minus"] = False

BASE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models")


def parse_log(path):
    """解析 train*.log，返回 (steps, success_n, crash, timeout, reward, steps_avg)."""
    steps, succ, fail, to, rew, steps_avg = [], [], [], [], [], []
    pat = re.compile(
        r"step=\s*(\d+) \| eps=([\d.]+) \| eval成功=(\d+)/(\d+) \(([\d.]+)%\) "
        r"\| 失败=(\d+) 超时=(\d+) \| 平均奖励=([-\d.]+) \| 平均步数=([\d.]+)")
    with open(path, encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = pat.search(line)
            if m:
                steps.append(int(m.group(1)))
                succ.append(int(m.group(3)))
                fail.append(int(m.group(6)))
                to.append(int(m.group(7)))
                rew.append(float(m.group(8)))
                steps_avg.append(float(m.group(9)))
    return steps, succ, fail, to, rew, steps_avg


def main():
    # ---- 图1：训练学习曲线 ----
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.2))
    for log, label, color in [("train35.log", "35动作 16障碍 (40万步)", "#d62728"),
                              ("train6.log", "35动作 6障碍 (40万步)", "#1f77b4")]:
        steps, succ, _, _, rew, _ = parse_log(os.path.join(BASE, log))
        if not steps:
            continue
        axs[0].plot(steps, np.array(succ) / 20 * 100, "-o", ms=3, lw=1.2,
                    color=color, label=label)
        axs[1].plot(steps, rew, "-o", ms=3, lw=1.2, color=color, label=label)
    axs[0].set_xlabel("训练步数"); axs[0].set_ylabel("评估成功率 (%)")
    axs[0].set_title("DQN 训练成功率（20回合评估）")
    axs[0].set_ylim(0, 100); axs[0].grid(alpha=0.3); axs[0].legend()
    axs[1].set_xlabel("训练步数"); axs[1].set_ylabel("平均奖励")
    axs[1].set_title("DQN 训练平均奖励")
    axs[1].grid(alpha=0.3); axs[1].legend()
    fig.suptitle("uav_dqn_nav DQN 训练曲线（Ubuntu 20.04 / ROS Noetic 虚拟机）", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(os.path.join(BASE, "training_curve.png"), dpi=150)
    print("图1 保存:", os.path.join(BASE, "training_curve.png"))

    # ---- 图2：飞行轨迹 ----
    traj_path = os.path.join(BASE, "traj_success.csv")
    obs_path = os.path.join(BASE, "traj_obs.csv")
    if not (os.path.exists(traj_path) and os.path.exists(obs_path)):
        print("跳过图2：缺少 traj_success.csv / traj_obs.csv")
        return
    traj = np.loadtxt(traj_path, delimiter=",", skiprows=1)
    obs = np.loadtxt(obs_path, delimiter=",", skiprows=1)
    fig, ax = plt.subplots(figsize=(7.5, 7.5))
    start, goal = traj[0, 3:6], traj[0, 6:9]
    ax.scatter(start[0], start[1], marker="o", s=120, c="green", zorder=5, label="起点")
    ax.scatter(goal[0], goal[1], marker="*", s=260, c="red", zorder=5, label="目标 (半径0.4m)")
    for ox, oy in obs:
        ax.add_patch(plt.Circle((ox, oy), 0.6, color="gray", alpha=0.45, zorder=2))
    ax.add_patch(plt.Circle((goal[0], goal[1]), 0.4, color="red", alpha=0.15, zorder=3))
    ax.plot(traj[:, 0], traj[:, 1], "-", color="#1f77b4", lw=1.5, zorder=4,
            label="DQN 轨迹 (%d 步)" % len(traj))
    ax.plot(traj[-1, 0], traj[-1, 1], "s", color="blue", ms=8, label="终点")
    ax.set_xlabel("东 (m)"); ax.set_ylabel("北 (m)")
    ax.set_title("uav_dqn_nav DQN 导航轨迹（6 障碍，最近距离回合 %.2f m）"
                 % np.linalg.norm(traj[-1, :3] - goal))
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend(loc="upper left", fontsize=9)
    fig.savefig(os.path.join(BASE, "trajectory.png"), dpi=150)
    print("图2 保存:", os.path.join(BASE, "trajectory.png"))


if __name__ == "__main__":
    main()
