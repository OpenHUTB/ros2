#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分析 trajectory.csv，输出指标、图表与 GIF。"""

import argparse
import json
import math
import os

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter, FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# 中文字体回退
plt.rcParams["font.sans-serif"] = [
    "Noto Sans CJK JP", "Noto Sans CJK SC", "Droid Sans Fallback",
    "WenQuanYi Zen Hei", "SimHei", "Arial Unicode MS", "DejaVu Sans",
]
plt.rcParams["axes.unicode_minus"] = False


def load_csv(path):
    data = np.genfromtxt(path, delimiter=",", names=True)
    return data


def metrics(data):
    t = data["time"]
    x, y, z = data["x"], data["y"], data["z"]
    tx, ty, tz = data["tx"], data["ty"], data["tz"]
    wx, wy, wz = data["wind_x"], data["wind_y"], data["wind_z"]

    pos = np.column_stack([x, y, z])
    tgt = np.column_stack([tx, ty, tz])
    err = np.linalg.norm(pos - tgt, axis=1)
    err_xy = np.linalg.norm(pos[:, :2] - tgt[:, :2], axis=1)
    err_z = np.abs(pos[:, 2] - tgt[:, 2])

    # 轨迹长度
    diffs = np.diff(pos, axis=0)
    length = float(np.sum(np.linalg.norm(diffs, axis=1)))

    # 识别航点切换时刻：目标点发生跳变
    tgt_diff = np.linalg.norm(np.diff(tgt, axis=0), axis=1)
    switch_mask = np.concatenate([[False], tgt_diff > 0.5])
    # 稳定悬停样本：距离当前目标 < 0.3 m，且距上次切换 > 2 s
    last_switch_t = -10.0
    stable = np.zeros(len(t), dtype=bool)
    for i in range(len(t)):
        if switch_mask[i]:
            last_switch_t = t[i]
        if err[i] < 0.3 and (t[i] - last_switch_t) > 2.0:
            stable[i] = True

    # 悬停段：最后 5 秒
    tail = t >= (t[-1] - 5.0)
    hover_rms = float(np.sqrt(np.mean(err[tail & stable] ** 2))) if np.any(tail & stable) else float("nan")
    hover_rms_xy = float(np.sqrt(np.mean(err_xy[tail & stable] ** 2))) if np.any(tail & stable) else float("nan")
    hover_rms_z = float(np.sqrt(np.mean(err_z[tail & stable] ** 2))) if np.any(tail & stable) else float("nan")

    # 阵风段（默认 18~24s）稳定悬停误差
    gust = (t >= 18.0) & (t <= 24.0)
    gust_stable = gust & stable
    gust_rms = float(np.sqrt(np.mean(err[gust_stable] ** 2))) if np.any(gust_stable) else float("nan")
    pre_gust = (t >= 14.0) & (t < 18.0)
    pre_stable = pre_gust & stable
    pre_rms = float(np.sqrt(np.mean(err[pre_stable] ** 2))) if np.any(pre_stable) else float("nan")

    wind_speed = np.sqrt(wx ** 2 + wy ** 2 + wz ** 2)

    return {
        "duration_s": float(t[-1]),
        "samples": int(len(t)),
        "trajectory_length_m": round(length, 3),
        "max_transition_error_m": round(float(np.max(err)), 4),
        "max_hover_error_m": round(float(np.max(err[stable])), 4) if np.any(stable) else None,
        "hover_rms_error_m": round(hover_rms, 4),
        "hover_rms_xy_m": round(hover_rms_xy, 4),
        "hover_rms_z_m": round(hover_rms_z, 4),
        "gust_hover_rms_m": round(gust_rms, 4),
        "pre_gust_hover_rms_m": round(pre_rms, 4),
        "max_wind_speed_mps": round(float(np.max(wind_speed)), 3),
        "mean_wind_speed_mps": round(float(np.mean(wind_speed)), 3),
        "max_roll_deg": round(float(np.max(np.abs(data["roll"]))) * 180 / math.pi, 2),
        "max_pitch_deg": round(float(np.max(np.abs(data["pitch"]))) * 180 / math.pi, 2),
        "final_error_m": round(float(err[-1]), 4),
        "stable_samples": int(np.sum(stable)),
    }


def plot_trajectory(data, out_dir):
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(data["x"], data["y"], data["z"], label="轨迹", lw=1.2)
    tx, ty, tz = data["tx"], data["ty"], data["tz"]
    ax.plot(tx, ty, tz, "r--", label="航点", lw=1.5)
    ax.scatter([data["x"][0]], [data["y"][0]], [data["z"][0]],
               c="g", s=40, label="起点")
    ax.scatter([data["x"][-1]], [data["y"][-1]], [data["z"][-1]],
               c="k", s=40, marker="x", label="终点")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    ax.set_title("四旋翼航点飞行 3D 轨迹")
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "trajectory.png"), dpi=120)
    plt.close(fig)


def plot_position(data, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    t = data["time"]
    for ax, key, tkey, label in zip(axes, ["x", "y", "z"], ["tx", "ty", "tz"],
                                    ["X", "Y", "Z"]):
        ax.plot(t, data[key], label="实际")
        ax.plot(t, data[tkey], "r--", label="目标")
        ax.set_ylabel(f"{label} (m)")
        ax.grid(alpha=0.3); ax.legend(fontsize=8)
    axes[-1].set_xlabel("时间 (s)")
    fig.suptitle("位置跟踪（含阵风扰动，18~24 s）")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "position.png"), dpi=120)
    plt.close(fig)


def plot_error(data, out_dir):
    fig, ax = plt.subplots(figsize=(9, 4))
    t = data["time"]
    pos = np.column_stack([data["x"], data["y"], data["z"]])
    tgt = np.column_stack([data["tx"], data["ty"], data["tz"]])
    err = np.linalg.norm(pos - tgt, axis=1)
    err_xy = np.linalg.norm(pos[:, :2] - tgt[:, :2], axis=1)
    err_z = np.abs(pos[:, 2] - tgt[:, 2])
    ax.plot(t, err, label="总误差", lw=1)
    ax.plot(t, err_xy, label="XY 误差", lw=1, alpha=0.7)
    ax.plot(t, err_z, label="Z 误差", lw=1, alpha=0.7)
    ax.axvspan(18, 24, color="orange", alpha=0.15, label="阵风时段")
    ax.set_xlabel("时间 (s)"); ax.set_ylabel("误差 (m)")
    ax.set_title("航点跟踪误差（阵风 18~24 s 体现抗风扰能力）")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "error.png"), dpi=120)
    plt.close(fig)


def plot_attitude(data, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    t = data["time"]
    for ax, key, label in zip(axes, ["roll", "pitch", "yaw"], ["滚转", "俯仰", "偏航"]):
        ax.plot(t, data[key] * 180 / math.pi)
        ax.set_ylabel(f"{label} (°)")
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel("时间 (s)")
    fig.suptitle("姿态角（rad -> deg）")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "attitude.png"), dpi=120)
    plt.close(fig)


def plot_wind(data, out_dir):
    fig, ax = plt.subplots(figsize=(9, 4))
    t = data["time"]
    ax.plot(t, data["wind_x"], label="Wx")
    ax.plot(t, data["wind_y"], label="Wy")
    ax.plot(t, np.sqrt(data["wind_x"]**2 + data["wind_y"]**2), label="|W|", lw=1.5)
    ax.axvspan(18, 24, color="orange", alpha=0.15, label="阵风")
    ax.set_xlabel("时间 (s)"); ax.set_ylabel("风速 (m/s)")
    ax.set_title("风场（恒定风 + 阵风 + 湍流）")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "wind.png"), dpi=120)
    plt.close(fig)


def plot_motors(data, out_dir):
    fig, ax = plt.subplots(figsize=(9, 4))
    t = data["time"]
    for i in range(1, 5):
        ax.plot(t, data[f"m{i}"], label=f"旋翼 {i}", lw=0.8)
    ax.set_xlabel("时间 (s)"); ax.set_ylabel("转速 (rad/s)")
    ax.set_title("四个旋翼转速")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "motors.png"), dpi=120)
    plt.close(fig)


def make_gif(data, out_dir):
    fig = plt.figure(figsize=(6, 5))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(data["x"], data["y"], data["z"], alpha=0.3, lw=0.8)
    line, = ax.plot([], [], [], "bo-", lw=1.5, markersize=4)
    ax.set_xlim(data["x"].min() - 0.5, data["x"].max() + 0.5)
    ax.set_ylim(data["y"].min() - 0.5, data["y"].max() + 0.5)
    ax.set_zlim(0, data["z"].max() + 0.5)
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    ax.set_title("四旋翼航点飞行动画")

    n = len(data["time"])
    step = max(1, n // 120)

    def update(i):
        idx = i * step
        line.set_data(data["x"][:idx], data["y"][:idx])
        line.set_3d_properties(data["z"][:idx])
        return line,

    anim = FuncAnimation(fig, update, frames=n // step, interval=50, blit=True)
    try:
        anim.save(os.path.join(out_dir, "trajectory.gif"), writer=PillowWriter(fps=20))
    except Exception as e:
        print("GIF 生成失败：", e)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", required=True)
    args = parser.parse_args()

    csv_path = os.path.join(args.data, "trajectory.csv")
    if not os.path.exists(csv_path):
        print("未找到 trajectory.csv")
        return
    data = load_csv(csv_path)
    m = metrics(data)

    with open(os.path.join(args.data, "metrics.json"), "w") as f:
        json.dump(m, f, indent=2, ensure_ascii=False)

    md = [
        "# 四旋翼串级 PID 航点悬停与抗风扰仿真指标",
        "",
        "| 指标 | 数值 |",
        "| --- | --- |",
    ]
    for k, v in m.items():
        md.append(f"| {k} | {v} |")
    with open(os.path.join(args.data, "metrics.md"), "w") as f:
        f.write("\n".join(md) + "\n")

    plot_trajectory(data, args.data)
    plot_position(data, args.data)
    plot_error(data, args.data)
    plot_attitude(data, args.data)
    plot_wind(data, args.data)
    plot_motors(data, args.data)
    make_gif(data, args.data)

    print(json.dumps(m, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
