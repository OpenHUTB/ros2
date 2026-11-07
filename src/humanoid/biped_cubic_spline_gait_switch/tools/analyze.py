#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分析双足多步态实验数据，生成指标、图表与 GIF。"""

import argparse
import csv
import json
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Circle, Rectangle

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG_ROOT)

from biped_cubic_spline_gait_switch.config_loader import (  # noqa: E402
    build_model, load_config)
from biped_cubic_spline_gait_switch.kinematics import BipedModel  # noqa: E402

JOINTS = ["left_hip", "left_knee", "left_ankle",
          "right_hip", "right_knee", "right_ankle"]


def setup_fonts():
    candidates = ["Noto Sans CJK JP", "Noto Sans CJK SC", "WenQuanYi Zen Hei",
                  "Droid Sans Fallback", "DejaVu Sans"]
    available = {f.name for f in font_manager.fontManager.ttflist}
    selected = [name for name in candidates if name in available]
    plt.rcParams["font.sans-serif"] = selected + ["DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False


def load_csv(path):
    rows = list(csv.DictReader(open(path, "r")))
    data = {}
    for key in rows[0].keys():
        if key == "gait":
            data[key] = [r[key] for r in rows]
        else:
            data[key] = np.array([float(r[key]) for r in rows])
    return data


def smooth(x, window=5):
    if window <= 1:
        return x
    kernel = np.ones(window) / window
    padded = np.pad(x, (window // 2, window // 2), mode="edge")
    return np.convolve(padded, kernel, mode="valid")[:len(x)]


def detect_switch_times(t, gait):
    """由步态名变化检测切换时刻。

    switch_event 是 200 Hz 控制器里的单周期脉冲，100 Hz 记录可能漏采；
    步态名在切换后保持不变，用它做检测更可靠。
    """
    names = np.asarray(gait, dtype=object)
    if len(names) < 2:
        return np.array([])
    change = np.where(names[1:] != names[:-1])[0] + 1
    return t[change]


def switch_windows(switch_times, duration=0.6):
    return [(float(t), float(t) + duration) for t in switch_times if t <= 200]


def window_indices(t, windows):
    mask = np.zeros(len(t), dtype=bool)
    for t0, t1 in windows:
        mask |= (t >= t0) & (t <= t1)
    return mask


def max_vel_jump(vel, t, windows):
    """各切换窗口内的最大相邻速度跳变。

    必须逐窗口计算：若直接对掩码后的数组做差分，窗口之间的长间隔会被
    误当成一次跳变。
    """
    jump = 0.0
    for t0, t1 in windows:
        idx = np.where((t >= t0) & (t <= t1))[0]
        if len(idx) > 1:
            seg = vel[idx]
            jump = max(jump, float(np.max(np.abs(np.diff(seg, axis=0)))))
    return jump


def compute_metrics(data, baseline=None):
    t = data["time"]
    q = np.stack([data["q%d" % i] for i in range(6)], axis=1)
    qd = np.stack([data["qd%d" % i] for i in range(6)], axis=1)
    qdes = np.stack([data["qdes%d" % i] for i in range(6)], axis=1)
    base_x = data["base_x"]

    start = t >= 1.0
    err = qdes - q
    tracking_rms = float(np.sqrt(np.mean(err[start] ** 2)))
    per_joint_rms = np.sqrt(np.mean(err[start] ** 2, axis=0)).tolist()

    switch_times = detect_switch_times(t, data["gait"])
    if len(switch_times) == 0:
        switch_times = t[data["switch_event"] > 0.5]
    windows = switch_windows(switch_times)
    mask = window_indices(t, windows)
    if np.sum(mask) < 5:
        mask = start

    dt = float(np.mean(np.diff(t)))
    q_s = np.stack([smooth(q[:, i]) for i in range(6)], axis=1)
    vel = np.gradient(q_s, dt, axis=0, edge_order=2)
    acc = np.gradient(vel, dt, axis=0, edge_order=2)
    jerk = np.gradient(acc, dt, axis=0, edge_order=2)

    max_vel = float(np.max(np.abs(vel)))
    max_acc = float(np.max(np.abs(acc[mask])))
    max_jerk = float(np.max(np.abs(jerk[mask])))
    vel_jump = max_vel_jump(vel, t, windows)

    stance = (data["lc"] > 0.5) | (data["rc"] > 0.5)
    margins = data["margin"][stance & (data["margin"] > -900)]
    margin_min = float(np.min(margins)) if len(margins) else float("nan")
    margin_mean = float(np.mean(margins)) if len(margins) else float("nan")
    flight_ratio = float(np.mean((data["lc"] < 0.5) & (data["rc"] < 0.5)))

    left_steps = int(np.sum(np.diff((data["lc"] > 0.5).astype(int)) > 0))
    right_steps = int(np.sum(np.diff((data["rc"] > 0.5).astype(int)) > 0))
    distance = float(base_x[-1] - base_x[0])
    moving = data["base_v"] > 0.01
    avg_speed = float(np.mean(data["base_v"][moving])) if np.any(moving) else 0.0

    gaits = {}
    for name in sorted(set(data["gait"])):
        idx = np.array([g == name for g in data["gait"]])
        g_margin = data["margin"][idx & (data["margin"] > -900)]
        g_err = err[idx]
        gaits[name] = {
            "samples": int(np.sum(idx)),
            "margin_min": float(np.min(g_margin)) if len(g_margin) else None,
            "tracking_rms": float(np.sqrt(np.mean(g_err ** 2))) if len(g_err) else None,
        }

    metrics = {
        "duration_s": float(t[-1] - t[0]),
        "samples": int(len(t)),
        "switch_count": int(len(switch_times)),
        "switch_times": [round(float(v), 3) for v in switch_times],
        "distance_m": round(distance, 3),
        "avg_speed_mps": round(avg_speed, 3),
        "tracking_rms_rad": round(tracking_rms, 4),
        "per_joint_rms_rad": [round(v, 4) for v in per_joint_rms],
        "max_joint_vel_rad_s": round(max_vel, 3),
        "max_joint_acc_rad_s2_switch": round(max_acc, 2),
        "max_joint_jerk_rad_s3_switch": round(max_jerk, 1),
        "max_vel_jump_rad_s": round(vel_jump, 3),
        "zmp_margin_min_m": round(margin_min, 4),
        "zmp_margin_mean_m": round(margin_mean, 4),
        "flight_ratio": round(flight_ratio, 3),
        "step_count": left_steps + right_steps,
        "gaits": gaits,
    }

    if baseline is not None:
        bt = baseline["time"]
        bq = np.stack([baseline["q%d" % i] for i in range(6)], axis=1)
        b_switch = detect_switch_times(bt, baseline["gait"])
        if len(b_switch) == 0:
            b_switch = bt[baseline["switch_event"] > 0.5]
        b_mask = window_indices(bt, switch_windows(b_switch))
        b_dt = float(np.mean(np.diff(bt)))
        b_s = np.stack([smooth(bq[:, i]) for i in range(6)], axis=1)
        b_vel = np.gradient(b_s, b_dt, axis=0, edge_order=2)
        b_acc = np.gradient(b_vel, b_dt, axis=0, edge_order=2)
        b_jerk = np.gradient(b_acc, b_dt, axis=0, edge_order=2)
        metrics["baseline"] = {
            "max_joint_acc_rad_s2_switch": round(float(np.max(np.abs(b_acc[b_mask]))), 2),
            "max_joint_jerk_rad_s3_switch": round(float(np.max(np.abs(b_jerk[b_mask]))), 1),
            "max_vel_jump_rad_s": round(max_vel_jump(b_vel, bt, switch_windows(b_switch)), 3),
        }
        if metrics["baseline"]["max_joint_jerk_rad_s3_switch"] > 0:
            metrics["jerk_reduction_percent"] = round(
                100.0 * (1.0 - metrics["max_joint_jerk_rad_s3_switch"]
                         / metrics["baseline"]["max_joint_jerk_rad_s3_switch"]), 1)

    return metrics, switch_times, vel, acc, jerk


def shade_switches(ax, switch_times, color="#f4a261", alpha=0.25):
    for ts in switch_times:
        ax.axvspan(ts, ts + 0.6, color=color, alpha=alpha, linewidth=0)


def plot_joints(data, switch_times, out):
    t = data["time"]
    fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
    for i, ax in enumerate(axes.ravel()):
        ax.plot(t, data["qdes%d" % i], color="#264653", lw=1.2, label="期望")
        ax.plot(t, data["q%d" % i], color="#e76f51", lw=1.0, ls="--", label="实际")
        shade_switches(ax, switch_times)
        ax.set_title(JOINTS[i])
        ax.set_ylabel("角度 (rad)")
        ax.grid(alpha=0.3)
    axes[0, 0].legend(loc="upper right", fontsize=8)
    for ax in axes[-1, :]:
        ax.set_xlabel("时间 (s)")
    fig.suptitle("多步态关节轨迹（橙色区间为三次样条切换过渡）")
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def plot_velocity(data, switch_times, out):
    t = data["time"]
    fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
    for i, ax in enumerate(axes.ravel()):
        ax.plot(t, data["qd%d" % i], color="#2a9d8f", lw=1.1)
        shade_switches(ax, switch_times)
        ax.set_title(JOINTS[i])
        ax.set_ylabel("角速度 (rad/s)")
        ax.grid(alpha=0.3)
    for ax in axes[-1, :]:
        ax.set_xlabel("时间 (s)")
    fig.suptitle("关节角速度")
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def plot_jerk(data, switch_times, jerk, metrics, out):
    t = data["time"]
    norm = np.max(np.abs(jerk), axis=1)
    fig, ax = plt.subplots(figsize=(12, 4.2))
    ax.plot(t, norm, color="#9b2226", lw=1.0, label="|jerk|∞（实际关节）")
    shade_switches(ax, switch_times)
    ax.set_yscale("log")
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("加加速度 (rad/s³)")
    ax.set_title("切换过程加加速度（对数坐标，橙色为 0.6 s 过渡窗口）")
    ax.grid(alpha=0.3, which="both")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def plot_zmp(data, switch_times, out):
    t = data["time"]
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True,
                             gridspec_kw={"height_ratios": [2, 1]})
    ax = axes[0]
    ax.plot(t, data["com_x"], color="#264653", lw=1.2, label="质心 x")
    ax.plot(t, data["zmp_x"], color="#e76f51", lw=1.0, label="ZMP x")
    lf, rf = data["lf_x"], data["rf_x"]
    lc, rc = data["lc"] > 0.5, data["rc"] > 0.5
    ax.scatter(lf[lc], np.full(np.sum(lc), -0.05), s=6, color="#2a9d8f", label="左脚支撑")
    ax.scatter(rf[rc], np.full(np.sum(rc), -0.08), s=6, color="#f4a261", label="右脚支撑")
    shade_switches(ax, switch_times)
    ax.set_ylabel("x (m)")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)
    ax.set_title("质心与 ZMP（ZMP 由期望质心轨迹计算）")

    ax2 = axes[1]
    ax2.plot(t, data["margin"], color="#457b9d", lw=1.0)
    ax2.axhline(0.0, color="gray", ls=":", lw=1.0)
    shade_switches(ax2, switch_times)
    ax2.set_xlabel("时间 (s)")
    ax2.set_ylabel("ZMP 裕度 (m)")
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def plot_feet(data, out):
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(data["lf_x"], data["lf_z"], color="#2a9d8f", lw=0.8, label="左脚")
    ax.plot(data["rf_x"], data["rf_z"], color="#f4a261", lw=0.8, label="右脚")
    ax.axhline(0.0, color="gray", lw=1)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("踝高 (m)")
    ax.set_title("双脚踏地/摆动轨迹")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def plot_timeline(data, switch_times, out):
    t = data["time"]
    names = list(dict.fromkeys(data["gait"]))
    colors = {"stand": "#8ecae6", "walk": "#219ebc", "fast_walk": "#ffb703",
              "run": "#fb8500"}
    fig, ax = plt.subplots(figsize=(12, 2.4))
    for i, name in enumerate(names):
        mask = np.array([g == name for g in data["gait"]])
        ax.fill_between(t, 0, 1, where=mask, color=colors.get(name, "#cccccc"),
                        step="mid", label=name)
    for ts in switch_times:
        ax.axvline(ts, color="#6a040f", ls="--", lw=1.0)
    ax.set_yticks([])
    ax.set_xlabel("时间 (s)")
    ax.set_title("步态调度时间线（虚线为切换时刻）")
    ax.legend(ncol=4, loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def plot_comparison(metrics, out):
    base = metrics.get("baseline", {})
    labels = ["max jerk", "max accel", "vel jump"]
    spline = [metrics["max_joint_jerk_rad_s3_switch"],
              metrics["max_joint_acc_rad_s2_switch"],
              metrics["max_vel_jump_rad_s"]]
    direct = [base.get("max_joint_jerk_rad_s3_switch", 0),
              base.get("max_joint_acc_rad_s2_switch", 0),
              base.get("max_vel_jump_rad_s", 0)]
    x = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=(8, 4.5))
    width = 0.35
    b1 = ax.bar(x - width / 2, spline, width, label="三次样条切换", color="#2a9d8f")
    b2 = ax.bar(x + width / 2, direct, width, label="直接切换", color="#e76f51")
    ax.set_yscale("log")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("数值（对数坐标）")
    ax.set_title("步态切换平滑性对比")
    ax.grid(alpha=0.3, axis="y", which="both")
    ax.legend()
    for bars in (b1, b2):
        for bar in bars:
            h = bar.get_height()
            ax.text(bar.get_x() + bar.get_width() / 2, h, "%.1f" % h,
                    ha="center", va="bottom", fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def render_gif(data, model, out, limit=280):
    t = data["time"]
    n = len(t)
    stride = max(1, n // limit)
    indices = list(range(0, n, stride))

    fig, ax = plt.subplots(figsize=(5.6, 3.2))
    fig.tight_layout()

    def draw_frame(idx):
        ax.clear()
        base_x = data["base_x"][idx]
        hip_z = data["hip_z"][idx]
        lean = data["lean"][idx]
        q = [data["q%d" % i][idx] for i in range(6)]
        pose = model.pose(base_x, hip_z, lean, q)

        ax.plot([base_x - 1.2, base_x + 1.2], [0, 0], color="#6c757d", lw=1.5)
        for side, color in (("left", "#2a9d8f"), ("right", "#f4a261")):
            leg = pose[side]
            pts = np.array([leg["hip"], leg["knee"], leg["ankle"]])
            ax.plot(pts[:, 0], pts[:, 1], "-o", color=color, lw=2.2, ms=3)
            foot = np.array([leg["heel"], leg["toe"]])
            ax.plot(foot[:, 0], foot[:, 1], color=color, lw=2.0)
        hip = pose["hip"]
        top = pose["torso_top"]
        ax.plot([hip[0], top[0]], [hip[1], top[1]], color="#264653", lw=3)
        ax.add_patch(Circle((top[0], top[1] + 0.08), 0.075, color="#264653"))
        ax.set_xlim(base_x - 1.2, base_x + 1.2)
        ax.set_ylim(-0.15, 1.5)
        ax.set_aspect("equal")
        ax.axis("off")
        ax.set_title("t = %.1f s   %s" % (data["time"][idx], data["gait"][idx]))

    frames = []
    for idx in indices:
        draw_frame(idx)
        fig.canvas.draw()
        image = np.asarray(fig.canvas.buffer_rgba())
        frames.append(image)
    plt.close(fig)

    from PIL import Image
    images = [Image.fromarray(frame[:, :, :3]).convert("P", palette=Image.ADAPTIVE)
              for frame in frames]
    images[0].save(out, save_all=True, append_images=images[1:],
                   duration=90, loop=0, optimize=False, disposal=2)


def write_markdown(metrics, out):
    lines = ["# 实验指标", ""]
    lines.append("| 指标 | 数值 |")
    lines.append("| --- | --- |")
    lines.append("| 仿真时长 | %.2f s |" % metrics["duration_s"])
    lines.append("| 切换次数 | %d |" % metrics["switch_count"])
    lines.append("| 前进距离 | %.3f m |" % metrics["distance_m"])
    lines.append("| 平均前进速度 | %.3f m/s |" % metrics["avg_speed_mps"])
    lines.append("| 关节跟踪 RMS | %.4f rad |" % metrics["tracking_rms_rad"])
    lines.append("| 切换窗口最大 jerk | %.1f rad/s³ |" % metrics["max_joint_jerk_rad_s3_switch"])
    lines.append("| 切换窗口最大角加速度 | %.2f rad/s² |" % metrics["max_joint_acc_rad_s2_switch"])
    lines.append("| ZMP 最小裕度 | %.4f m |" % metrics["zmp_margin_min_m"])
    lines.append("| 腾空比例 | %.3f |" % metrics["flight_ratio"])
    lines.append("| 步数 | %d |" % metrics["step_count"])
    if "baseline" in metrics:
        lines.append("| 直接切换最大 jerk | %.1f rad/s³ |"
                     % metrics["baseline"]["max_joint_jerk_rad_s3_switch"])
        if "jerk_reduction_percent" in metrics:
            lines.append("| jerk 降低 | %.1f%% |" % metrics["jerk_reduction_percent"])
    lines.append("")
    lines.append("## 分步态")
    lines.append("")
    lines.append("| 步态 | 采样数 | ZMP 最小裕度 (m) | 跟踪 RMS (rad) |")
    lines.append("| --- | --- | --- | --- |")
    for name, g in metrics["gaits"].items():
        lines.append("| %s | %d | %s | %s |" % (
            name, g["samples"],
            "-" if g["margin_min"] is None else "%.4f" % g["margin_min"],
            "-" if g["tracking_rms"] is None else "%.4f" % g["tracking_rms"]))
    open(out, "w").write("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", default="results/run_spline")
    parser.add_argument("--baseline", default="results/run_direct")
    parser.add_argument("--config", default=os.path.join(PKG_ROOT, "config", "params.yaml"))
    args = parser.parse_args()

    setup_fonts()
    model = build_model(load_config(args.config))
    data = load_csv(os.path.join(args.data, "trajectory.csv"))
    baseline = None
    if os.path.exists(os.path.join(args.baseline, "trajectory.csv")):
        baseline = load_csv(os.path.join(args.baseline, "trajectory.csv"))

    metrics, switch_times, vel, acc, jerk = compute_metrics(data, baseline)
    with open(os.path.join(args.data, "metrics.json"), "w") as f:
        json.dump(metrics, f, ensure_ascii=False, indent=2)
    write_markdown(metrics, os.path.join(args.data, "metrics.md"))

    plot_joints(data, switch_times, os.path.join(args.data, "joints.png"))
    plot_velocity(data, switch_times, os.path.join(args.data, "velocity.png"))
    plot_jerk(data, switch_times, jerk, metrics, os.path.join(args.data, "jerk.png"))
    plot_zmp(data, switch_times, os.path.join(args.data, "zmp.png"))
    plot_feet(data, os.path.join(args.data, "feet.png"))
    plot_timeline(data, switch_times, os.path.join(args.data, "gait_timeline.png"))
    if baseline is not None:
        plot_comparison(metrics, os.path.join(args.data, "comparison.png"))
    render_gif(data, model, os.path.join(args.data, "walking.gif"))

    print(json.dumps(metrics, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
