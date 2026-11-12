#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双侧墙通道居中 PD 巡航离线仿真（不依赖 ROS 2 / Gazebo / MuJoCo）。

在仓库根目录运行：
    python3 src/ground/diff_drive_corridor_centering/offline_sim.py

脚本用 2D 差速运动学积分模拟车体在直通道内的运动，测距由通道几何直接
推算，控制器调用与 ROS 2 节点同一个 ``centering.py`` 核心。运行结束后：

- 在终端打印时长、路径长度、横向误差等指标；
- 在 ``docs/img/ground/`` 下生成三张效果图：轨迹、左右墙距离、速度指令。
"""

from __future__ import annotations

import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# 允许从模块目录直接运行：把功能包目录加入导入路径。
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from diff_drive_corridor_centering.centering import (  # noqa: E402
    CenteringParams,
    CorridorCenteringController,
    diff_drive_step,
)

CORRIDOR_HALF_WIDTH = 1.0
CORRIDOR_LENGTH = 14.0
DT = 0.05
MAX_TIME = 30.0
GOAL_X = 13.5


def run_sim() -> dict:
    params = CenteringParams()
    controller = CorridorCenteringController(params)

    pose = [0.3, 0.55, -0.18]  # x, y, yaw
    t = 0.0
    records = {"t": [], "x": [], "y": [], "yaw": [],
               "d_left": [], "d_right": [], "d_forward": [], "v": [], "w": [], "e": []}
    path_length = 0.0
    prev_xy = (pose[0], pose[1])

    while t < MAX_TIME and pose[0] < GOAL_X:
        d_left = CORRIDOR_HALF_WIDTH - pose[1]
        d_right = CORRIDOR_HALF_WIDTH + pose[1]
        d_forward = max(0.0, (CORRIDOR_LENGTH - pose[0]) /
                        max(math.cos(pose[2]), 0.2))

        result = controller.step(d_left, d_right, d_forward, t)
        v, w = result["v"], result["w"]

        new_pose = diff_drive_step(tuple(pose), v, w, DT)
        path_length += math.hypot(new_pose[0] - prev_xy[0],
                                 new_pose[1] - prev_xy[1])
        prev_xy = (new_pose[0], new_pose[1])
        pose = list(new_pose)
        t += DT

        records["t"].append(t)
        records["x"].append(pose[0])
        records["y"].append(pose[1])
        records["yaw"].append(pose[2])
        records["d_left"].append(d_left)
        records["d_right"].append(d_right)
        records["d_forward"].append(d_forward)
        records["v"].append(v)
        records["w"].append(w)
        records["e"].append(result["e"])

    records = {k: np.asarray(v) for k, v in records.items()}
    metrics = {
        "duration_s": round(float(records["t"][-1]), 2),
        "path_length_m": round(path_length, 2),
        "final_lateral_error_m": round(float(records["e"][-1]), 4),
        "mean_abs_error_m": round(float(np.mean(np.abs(records["e"]))), 4),
        "max_abs_error_m": round(float(np.max(np.abs(records["e"]))), 4),
        "min_forward_clearance_m": round(float(np.min(records["d_forward"])), 3),
        "samples": int(len(records["t"])),
    }
    return {"records": records, "metrics": metrics}


def _output_dir() -> str:
    # offline_sim.py -> 模块目录 -> ground -> src -> 仓库根
    repo_root = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))
    out = os.path.join(repo_root, "docs", "img", "ground")
    os.makedirs(out, exist_ok=True)
    return out


def make_plots(records: dict, out_dir: str) -> list[str]:
    t = records["t"]
    paths = []

    # 1. 俯视轨迹：通道两墙 + 车体路径。
    fig, ax = plt.subplots(figsize=(10, 3.2))
    ax.axhline(CORRIDOR_HALF_WIDTH, color="gray", linestyle="--",
               label="left wall")
    ax.axhline(-CORRIDOR_HALF_WIDTH, color="gray", linestyle="--",
               label="right wall")
    ax.plot(records["x"], records["y"], color="#1f77b4", linewidth=2,
            label="robot path")
    ax.scatter([records["x"][0]], [records["y"][0]], color="green",
               zorder=5, label="start")
    ax.scatter([records["x"][-1]], [records["y"][-1]], color="red",
               zorder=5, label="end")
    ax.axhline(0.0, color="lightgray", linewidth=0.8)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Corridor centering trajectory (top view)")
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    p1 = os.path.join(out_dir, "diff_drive_corridor_centering_trajectory.png")
    fig.savefig(p1, dpi=120)
    plt.close(fig)
    paths.append(p1)

    # 2. 左右墙距离随时间。
    fig, ax = plt.subplots(figsize=(10, 3.2))
    ax.plot(t, records["d_left"], color="#2ca02c", label="d_left")
    ax.plot(t, records["d_right"], color="#ff7f0e", label="d_right")
    ax.axhline(CORRIDOR_HALF_WIDTH, color="gray", linestyle="--",
               label="center reference")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("distance (m)")
    ax.set_title("Left / right wall distance over time")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    p2 = os.path.join(out_dir, "diff_drive_corridor_centering_wall_distance.png")
    fig.savefig(p2, dpi=120)
    plt.close(fig)
    paths.append(p2)

    # 3. 速度指令：线速度 / 角速度。
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 5), sharex=True)
    ax1.plot(t, records["v"], color="#1f77b4", label="linear v")
    ax1.set_ylabel("v (m/s)")
    ax1.set_title("Commanded velocity")
    ax1.legend(fontsize=8)
    ax1.grid(alpha=0.3)
    ax2.plot(t, records["w"], color="#d62728", label="angular w")
    ax2.set_ylabel("w (rad/s)")
    ax2.set_xlabel("t (s)")
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    p3 = os.path.join(out_dir, "diff_drive_corridor_centering_cmd_vel.png")
    fig.savefig(p3, dpi=120)
    plt.close(fig)
    paths.append(p3)

    return paths


def main() -> None:
    result = run_sim()
    records, metrics = result["records"], result["metrics"]
    out_dir = _output_dir()
    paths = make_plots(records, out_dir)

    print("=== 离线仿真指标 ===")
    for k, v in metrics.items():
        print(f"{k}: {v}")
    print("=== 效果图 ===")
    for p in paths:
        print(p)


if __name__ == "__main__":
    main()
