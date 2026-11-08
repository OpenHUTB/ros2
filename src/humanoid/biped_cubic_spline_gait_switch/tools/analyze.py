#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分析双足多步态实验数据，生成指标、图表与 GIF。

记录节点的 CSV 已按固定 100 Hz 网格打时间戳；分析时再重采样到同一均匀网格，
并在命令侧位置 qdes 上差分得到加速度/jerk/相邻采样速度差（不依赖实际关节的伺服滤波）。
"""

import argparse
import csv
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Circle

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG_ROOT)

from biped_cubic_spline_gait_switch.config_loader import (  # noqa: E402
    build_model, load_config)
from biped_cubic_spline_gait_switch.kinematics import BipedModel  # noqa: E402

JOINTS = ["left_hip", "left_knee", "left_ankle",
          "right_hip", "right_knee", "right_ankle"]
DEFAULT_ANALYSIS_RATE = 100.0
DEFAULT_SWITCH_WINDOW = 0.6


def load_analysis_params(data_dir, config_path):
    """从记录的 run_info.json 与 params.yaml 读取分析网格和过渡窗口。

    记录频率取 run_info.json（logger 的实际发布频率），过渡时长取
    params.yaml 的 control.transition_time；两者都不再写死在代码里。
    """
    rate = DEFAULT_ANALYSIS_RATE
    run_info = os.path.join(data_dir, "run_info.json")
    if os.path.exists(run_info):
        with open(run_info, "r") as f:
            info = json.load(f)
        rate = float(info.get("rate", rate))
    window = DEFAULT_SWITCH_WINDOW
    if config_path and os.path.exists(config_path):
        cfg = load_config(config_path)
        window = float(cfg.get("control", {}).get("transition_time", window))
    return rate, window


def setup_fonts():
    candidates = ["Noto Sans CJK JP", "Noto Sans CJK SC", "WenQuanYi Zen Hei",
                  "Droid Sans Fallback", "DejaVu Sans"]
    available = {f.name for f in font_manager.fontManager.ttflist}
    selected = [name for name in candidates if name in available]
    plt.rcParams["font.sans-serif"] = selected + ["DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False


def load_csv(path):
    with open(path, "r", newline="") as f:
        rows = [r for r in csv.DictReader(f) if r.get("gait", "").strip()]
    data = {}
    for key in rows[0].keys():
        if key == "gait":
            data[key] = [r[key] for r in rows]
        else:
            data[key] = np.array([float(r[key]) for r in rows])
    return data


def detect_switch_times(t, gait):
    """由步态名变化检测切换时刻；空步态名（历史数据首行）不计入。"""
    names = np.asarray(gait, dtype=object)
    if len(names) < 2:
        return np.array([])
    valid = np.array([isinstance(n, str) and n != "" for n in names])
    change = np.where((names[1:] != names[:-1]) & valid[1:] & valid[:-1])[0] + 1
    return t[change]


def switch_windows(switch_times, duration):
    return [(float(t), float(t) + float(duration)) for t in switch_times]


def window_indices(t, windows):
    mask = np.zeros(len(t), dtype=bool)
    for t0, t1 in windows:
        mask |= (t >= t0) & (t <= t1)
    return mask


def resample_data(data, rate):
    """把数值列线性插值到等间隔网格（接触量随后按 0.5 阈值还原）。"""
    t = np.asarray(data["time"], dtype=float)
    grid = np.arange(t[0], t[-1] + 1e-9, 1.0 / rate)
    out = {"time": grid}
    for key, value in data.items():
        if key in ("time", "gait"):
            continue
        out[key] = np.interp(grid, t, np.asarray(value, dtype=float))
    return out


def switch_steps(values, t, switch_times):
    """切换前后相邻样本差：最后一个切换前样本与第一个切换后样本之差。

    这是 100 Hz 采样下的表观差，用于说明相邻采样的连续性强弱；真实左右
    极限差由 pytest 对规划器解析导数的逐次断言验证。
    """
    steps = {}
    for ts in switch_times:
        idx = int(np.searchsorted(t, ts, side="left"))
        if idx <= 0 or idx >= len(t):
            continue
        steps[round(float(ts), 3)] = float(np.max(np.abs(values[idx] - values[idx - 1])))
    return steps


def max_adjacent_diff(values, t, windows):
    """各切换窗口内的最大相邻采样差（不是左右极限差）。"""
    diff = 0.0
    for t0, t1 in windows:
        idx = np.where((t >= t0) & (t <= t1))[0]
        if len(idx) > 1:
            seg = values[idx]
            diff = max(diff, float(np.max(np.abs(np.diff(seg, axis=0)))))
    return diff


def max_adjacent_diff_at(values, t, windows):
    """最大相邻采样差出现的位置（样本时刻）。"""
    best = 0.0
    best_t = None
    for t0, t1 in windows:
        idx = np.where((t >= t0) & (t <= t1))[0]
        if len(idx) > 1:
            diff = np.abs(np.diff(values[idx], axis=0)).max(axis=1)
            k = int(np.argmax(diff))
            value = float(diff[k])
            if value > best:
                best = value
                best_t = float(t[idx[k + 1]])
    return best, best_t


def command_smoothness(series, windows, rate, switch_times):
    """在命令侧位置 qdes 的均匀网格上计算加速度/jerk/速度差。

    同时给出 qddes（控制器发布的命令速度）的相邻采样差与切换时刻左右
    极限差，用于验证 C1 连续。
    """
    dt = 1.0 / rate
    t = series["time"]
    qdes = np.stack([series["qdes%d" % i] for i in range(6)], axis=1)
    qddes = np.stack([series["qddes%d" % i] for i in range(6)], axis=1)
    vel = np.gradient(qdes, dt, axis=0, edge_order=2)
    acc = np.gradient(vel, dt, axis=0, edge_order=2)
    jerk = np.gradient(acc, dt, axis=0, edge_order=2)
    mask = window_indices(t, windows)
    if np.sum(mask) < 5:
        mask = t >= 1.0
    steps = switch_steps(qddes, t, switch_times)
    sample_cmd_diff, sample_cmd_diff_at = max_adjacent_diff_at(qddes, t, windows)
    return {
        "vel": vel,
        "acc": acc,
        "jerk": jerk,
        "max_vel": float(np.max(np.abs(vel))),
        "max_acc": float(np.max(np.abs(acc[mask]))),
        "max_jerk": float(np.max(np.abs(jerk[mask]))),
        "sample_vel_diff": max_adjacent_diff(vel, t, windows),
        "sample_cmd_vel_diff": sample_cmd_diff,
        "sample_cmd_vel_diff_at": sample_cmd_diff_at,
        "cmd_vel_steps": steps,
        "max_cmd_vel_step": max(steps.values()) if steps else 0.0,
    }


def grid_sensitivity(data, switch_times, rate, window, factors=(1, 2, 4)):
    """不同分析网格下的 jerk / 角加速度，用于说明差分指标的分辨率依赖。"""
    rates = [rate * f for f in factors]
    jerk = []
    acc = []
    for r in rates:
        series = resample_data(data, r)
        smooth = command_smoothness(
            series, switch_windows(switch_times, window), r, switch_times)
        jerk.append(round(smooth["max_jerk"], 1))
        acc.append(round(smooth["max_acc"], 2))
    return {"rates_hz": [round(r, 3) for r in rates],
            "max_jerk_rad_s3": jerk,
            "max_acc_rad_s2": acc}


def compute_metrics(data, baseline=None, rate=DEFAULT_ANALYSIS_RATE,
                    window=DEFAULT_SWITCH_WINDOW, baseline_rate=None):
    raw_t = np.asarray(data["time"], dtype=float)
    switch_times = detect_switch_times(raw_t, data["gait"])
    windows = switch_windows(switch_times, window)

    series = resample_data(data, rate)
    smooth = command_smoothness(series, windows, rate, switch_times)

    t = series["time"]
    q = np.stack([series["q%d" % i] for i in range(6)], axis=1)
    qdes = np.stack([series["qdes%d" % i] for i in range(6)], axis=1)
    base_x = series["base_x"]

    start = t >= 1.0
    err = qdes - q
    tracking_rms = float(np.sqrt(np.mean(err[start] ** 2)))
    per_joint_rms = np.sqrt(np.mean(err[start] ** 2, axis=0)).tolist()

    lc = series["lc"] > 0.5
    rc = series["rc"] > 0.5
    margin = series["margin"]
    margin = margin[np.isfinite(margin)]
    margin_min = float(np.min(margin)) if len(margin) else float("nan")
    margin_mean = float(np.mean(margin)) if len(margin) else float("nan")
    flight_ratio = float(np.mean(~lc & ~rc))

    raw_gait = data["gait"]
    same_gait = np.array([raw_gait[i] == raw_gait[i - 1]
                          for i in range(1, len(raw_gait))])

    def count_steps(contact):
        rises = np.diff(contact.astype(int)) > 0
        # 排除步态切换边界处接触状态跳变产生的假上升沿
        return int(np.sum(rises & same_gait))

    left_steps = count_steps(np.asarray(data["lc"]) > 0.5)
    right_steps = count_steps(np.asarray(data["rc"]) > 0.5)
    distance = float(base_x[-1] - base_x[0])
    duration = float(t[-1] - t[0])
    avg_speed = distance / max(duration, 1e-9)
    moving = series["base_v"] > 0.01
    avg_speed_moving = float(np.mean(series["base_v"][moving])) if np.any(moving) else 0.0

    gaits = {}
    q_raw = np.stack([np.asarray(data["q%d" % i]) for i in range(6)], axis=1)
    qdes_raw = np.stack([np.asarray(data["qdes%d" % i]) for i in range(6)], axis=1)
    err_raw = qdes_raw - q_raw
    for name in sorted(set(data["gait"])):
        idx = np.array([g == name for g in data["gait"]])
        g_margin = np.asarray(data["margin"])[idx]
        g_margin = g_margin[np.isfinite(g_margin)]
        g_err = err_raw[idx]
        gaits[name] = {
            "samples": int(np.sum(idx)),
            "margin_min": float(np.min(g_margin)) if len(g_margin) else None,
            "tracking_rms": float(np.sqrt(np.mean(g_err ** 2))),
        }

    metrics = {
        "duration_s": float(raw_t[-1] - raw_t[0]),
        "analysis_rate_hz": rate,
        "switch_window_s": window,
        "samples": int(len(raw_t)),
        "switch_count": int(len(switch_times)),
        "switch_times": [round(float(v), 3) for v in switch_times],
        "distance_m": round(distance, 3),
        "avg_speed_mps": round(avg_speed, 3),
        "avg_speed_moving_mps": round(avg_speed_moving, 3),
        "tracking_rms_rad": round(tracking_rms, 4),
        "per_joint_rms_rad": [round(v, 4) for v in per_joint_rms],
        "max_joint_vel_rad_s": round(smooth["max_vel"], 3),
        "max_joint_acc_rad_s2_switch": round(smooth["max_acc"], 2),
        "max_joint_jerk_rad_s3_switch": round(smooth["max_jerk"], 1),
        "max_sample_vel_diff_rad_s": round(smooth["sample_vel_diff"], 3),
        "max_sample_cmd_vel_diff_rad_s": round(smooth["sample_cmd_vel_diff"], 3),
        "max_sample_cmd_vel_diff_at_s": (None if smooth["sample_cmd_vel_diff_at"] is None
                                         else round(smooth["sample_cmd_vel_diff_at"], 3)),
        "max_switch_cmd_vel_step_rad_s": round(smooth["max_cmd_vel_step"], 6),
        "switch_cmd_vel_steps_rad_s": {
            "%.3f" % k: round(v, 6) for k, v in smooth["cmd_vel_steps"].items()},
        "zmp_margin_min_m": round(margin_min, 4),
        "zmp_margin_mean_m": round(margin_mean, 4),
        "flight_ratio": round(flight_ratio, 3),
        "step_count": left_steps + right_steps,
        "metric_source": "command_qdes",
        "gaits": gaits,
    }

    if baseline is not None:
        b_rate = baseline_rate or rate
        bt = np.asarray(baseline["time"], dtype=float)
        b_switch = detect_switch_times(bt, baseline["gait"])
        b_series = resample_data(baseline, b_rate)
        b_smooth = command_smoothness(
            b_series, switch_windows(b_switch, window), b_rate, b_switch)
        metrics["baseline"] = {
            "switch_count": int(len(b_switch)),
            "switch_times": [round(float(v), 3) for v in b_switch],
            "max_joint_acc_rad_s2_switch": round(b_smooth["max_acc"], 2),
            "max_joint_jerk_rad_s3_switch": round(b_smooth["max_jerk"], 1),
            "max_sample_vel_diff_rad_s": round(b_smooth["sample_vel_diff"], 3),
            "max_sample_cmd_vel_diff_rad_s": round(b_smooth["sample_cmd_vel_diff"], 3),
            "max_sample_cmd_vel_diff_at_s": (None if b_smooth["sample_cmd_vel_diff_at"] is None
                                             else round(b_smooth["sample_cmd_vel_diff_at"], 3)),
            "max_switch_cmd_vel_step_rad_s": round(b_smooth["max_cmd_vel_step"], 6),
            "switch_cmd_vel_steps_rad_s": {
                "%.3f" % k: round(v, 6) for k, v in b_smooth["cmd_vel_steps"].items()},
        }
        if metrics["baseline"]["max_joint_jerk_rad_s3_switch"] > 0:
            metrics["jerk_reduction_percent"] = round(
                100.0 * (1.0 - metrics["max_joint_jerk_rad_s3_switch"]
                         / metrics["baseline"]["max_joint_jerk_rad_s3_switch"]), 1)
        if metrics["baseline"]["max_joint_acc_rad_s2_switch"] > 0:
            metrics["acc_reduction_percent"] = round(
                100.0 * (1.0 - metrics["max_joint_acc_rad_s2_switch"]
                         / metrics["baseline"]["max_joint_acc_rad_s2_switch"]), 1)

        sensitivity = grid_sensitivity(data, switch_times, rate, window)
        b_sensitivity = grid_sensitivity(baseline, b_switch, b_rate, window)
        metrics["grid_sensitivity"] = {
            "rates_hz": sensitivity["rates_hz"],
            "spline": sensitivity,
            "direct": b_sensitivity,
        }

    return metrics, switch_times, t, smooth["vel"], smooth["acc"], smooth["jerk"]


def shade_switches(ax, switch_times, window, color="#f4a261", alpha=0.25):
    for ts in switch_times:
        ax.axvspan(ts, ts + window, color=color, alpha=alpha, linewidth=0)


def plot_joints(data, switch_times, out, window):
    t = data["time"]
    fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
    for i, ax in enumerate(axes.ravel()):
        ax.plot(t, data["qdes%d" % i], color="#264653", lw=1.2, label="期望")
        ax.plot(t, data["q%d" % i], color="#e76f51", lw=1.0, ls="--", label="实际")
        shade_switches(ax, switch_times, window)
        ax.set_title(JOINTS[i])
        ax.set_ylabel("角度 (rad)")
        ax.grid(alpha=0.3)
    axes[0, 0].legend(loc="upper right", fontsize=8)
    for ax in axes[-1, :]:
        ax.set_xlabel("时间 (s)")
    fig.suptitle("多步态关节轨迹（橙色区间为三次 Hermite 过渡）")
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def plot_velocity(data, switch_times, out, window):
    t = data["time"]
    fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
    for i, ax in enumerate(axes.ravel()):
        ax.plot(t, data["qddes%d" % i], color="#264653", lw=1.2, label="命令角速度")
        ax.plot(t, data["qd%d" % i], color="#2a9d8f", lw=1.0, ls="--", label="实际角速度")
        shade_switches(ax, switch_times, window)
        ax.set_title(JOINTS[i])
        ax.set_ylabel("角速度 (rad/s)")
        ax.grid(alpha=0.3)
    axes[0, 0].legend(loc="upper right", fontsize=8)
    for ax in axes[-1, :]:
        ax.set_xlabel("时间 (s)")
    fig.suptitle("关节角速度（命令值由 Hermite 解析导数给出）")
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def plot_jerk(t, switch_times, jerk, out, window, rate):
    norm = np.max(np.abs(jerk), axis=1)
    fig, ax = plt.subplots(figsize=(12, 4.2))
    ax.plot(t, norm, color="#9b2226", lw=1.0, label="|jerk|∞（命令侧 qdes 差分）")
    shade_switches(ax, switch_times, window)
    ax.set_yscale("log")
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("加加速度 (rad/s³)")
    ax.set_title("切换过程加加速度（%g Hz 均匀网格，橙色为 %.1f s 过渡窗口）"
                 % (rate, window))
    ax.grid(alpha=0.3, which="both")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def plot_zmp(data, switch_times, out, window):
    t = data["time"]
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True,
                             gridspec_kw={"height_ratios": [2, 1]})
    ax = axes[0]
    ax.plot(t, data["com_x"], color="#264653", lw=1.2, label="实际质心 x")
    ax.plot(t, data["zmp_x"], color="#e76f51", lw=1.0, label="ZMP x")
    lf, rf = data["lf_x"], data["rf_x"]
    lc, rc = data["lc"] > 0.5, data["rc"] > 0.5
    ax.scatter(lf[lc], np.full(np.sum(lc), -0.05), s=6, color="#2a9d8f", label="左脚支撑")
    ax.scatter(rf[rc], np.full(np.sum(rc), -0.08), s=6, color="#f4a261", label="右脚支撑")
    shade_switches(ax, switch_times, window)
    ax.set_ylabel("x (m)")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)
    ax.set_title("质心与 ZMP（ZMP 由期望质心轨迹计算）")

    ax2 = axes[1]
    finite = np.where(np.isfinite(data["margin"]), data["margin"], np.nan)
    ax2.plot(t, finite, color="#457b9d", lw=1.0)
    ax2.axhline(0.0, color="gray", ls=":", lw=1.0)
    shade_switches(ax2, switch_times, window)
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
    ax.set_title("双脚踏地/摆动轨迹（规划 qdes 的 FK）")
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
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))

    labels = ["max jerk", "max accel"]
    spline1 = [metrics["max_joint_jerk_rad_s3_switch"],
               metrics["max_joint_acc_rad_s2_switch"]]
    direct1 = [base.get("max_joint_jerk_rad_s3_switch", 0),
               base.get("max_joint_acc_rad_s2_switch", 0)]
    x = np.arange(len(labels))
    width = 0.35
    b1 = ax1.bar(x - width / 2, spline1, width, label="三次 Hermite 切换", color="#2a9d8f")
    b2 = ax1.bar(x + width / 2, direct1, width, label="直接切换", color="#e76f51")
    ax1.set_yscale("log")
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels)
    ax1.set_ylabel("数值（对数坐标）")
    ax1.set_title("差分指标（依赖分析网格）")
    ax1.grid(alpha=0.3, axis="y", which="both")
    ax1.legend()
    for bars in (b1, b2):
        for bar in bars:
            h = bar.get_height()
            ax1.text(bar.get_x() + bar.get_width() / 2, h, "%.1f" % h,
                     ha="center", va="bottom", fontsize=8)

    step = [metrics["max_switch_cmd_vel_step_rad_s"],
            base.get("max_switch_cmd_vel_step_rad_s", 0)]
    bars = ax2.bar(["Hermite", "直接"], step,
                   color=["#2a9d8f", "#e76f51"], width=0.55)
    ax2.set_ylabel("qddes 左右极限差 (rad/s)")
    ax2.set_title("切换时刻命令速度连续性（主结论）")
    ax2.grid(alpha=0.3, axis="y")
    for bar, value in zip(bars, step):
        ax2.text(bar.get_x() + bar.get_width() / 2, value, "%.3f" % value,
                 ha="center", va="bottom", fontsize=9)

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
    lines.append("| 平均前进速度（全程） | %.3f m/s |" % metrics["avg_speed_mps"])
    lines.append("| 平均前进速度（行进段） | %.3f m/s |" % metrics["avg_speed_moving_mps"])
    lines.append("| 关节跟踪 RMS | %.4f rad |" % metrics["tracking_rms_rad"])
    lines.append("| 切换窗口最大 jerk（命令侧 qdes） | %.1f rad/s³ |"
                 % metrics["max_joint_jerk_rad_s3_switch"])
    lines.append("| 切换窗口最大角加速度（命令侧 qdes） | %.2f rad/s² |"
                 % metrics["max_joint_acc_rad_s2_switch"])
    lines.append("| 切换窗口最大相邻采样速度差（qdes 差分） | %.3f rad/s |"
                 % metrics["max_sample_vel_diff_rad_s"])
    lines.append("| 切换窗口最大相邻采样命令速度差（qddes） | %.3f rad/s |"
                 % metrics["max_sample_cmd_vel_diff_rad_s"])
    if metrics.get("max_sample_cmd_vel_diff_at_s") is not None:
        lines.append("| 上述最大差出现时刻 | %.3f s |"
                     % metrics["max_sample_cmd_vel_diff_at_s"])
    lines.append("| 切换时刻 qddes 左右极限差（全部切换取最大） | %.6f rad/s |"
                 % metrics["max_switch_cmd_vel_step_rad_s"])
    lines.append("| ZMP 最小裕度（规划几何量） | %.4f m |" % metrics["zmp_margin_min_m"])
    lines.append("| 腾空比例（规划相位） | %.3f |" % metrics["flight_ratio"])
    lines.append("| 步数（规划相位） | %d |" % metrics["step_count"])
    if "baseline" in metrics:
        lines.append("| 直接切换最大 jerk | %.1f rad/s³ |"
                     % metrics["baseline"]["max_joint_jerk_rad_s3_switch"])
        lines.append("| 直接切换最大相邻采样命令速度差 | %.3f rad/s |"
                     % metrics["baseline"]["max_sample_cmd_vel_diff_rad_s"])
        lines.append("| 直接切换 qddes 左右极限差 | %.6f rad/s |"
                     % metrics["baseline"]["max_switch_cmd_vel_step_rad_s"])
        if "jerk_reduction_percent" in metrics:
            lines.append("| jerk 降低（仅 100 Hz 网格） | %.1f%% |"
                         % metrics["jerk_reduction_percent"])
        if "acc_reduction_percent" in metrics:
            lines.append("| 角加速度降低（仅 100 Hz 网格） | %.1f%% |"
                         % metrics["acc_reduction_percent"])
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
    sens = metrics.get("grid_sensitivity")
    if sens:
        rates = sens["rates_hz"]
        lines.append("")
        lines.append("## 指标分辨率敏感性")
        lines.append("")
        lines.append("差分指标依赖分析网格：网格越细，直接切换的位置阶跃与 "
                     "Hermite 过渡端点的加速度跳变都会使 jerk 增大，两者都不收敛；"
                     "因此主结论采用切换时刻 qddes 的左右极限差。")
        lines.append("")
        lines.append("| 网格 (Hz) | 直接 jerk | Hermite jerk | 直接 acc | Hermite acc |")
        lines.append("| --- | --- | --- | --- | --- |")
        for i, r in enumerate(rates):
            lines.append("| %g | %.1f | %.1f | %.2f | %.2f |" % (
                r, sens["direct"]["max_jerk_rad_s3"][i],
                sens["spline"]["max_jerk_rad_s3"][i],
                sens["direct"]["max_acc_rad_s2"][i],
                sens["spline"]["max_acc_rad_s2"][i]))
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
    rate, window = load_analysis_params(args.data, args.config)
    baseline = None
    baseline_rate = None
    if os.path.exists(os.path.join(args.baseline, "trajectory.csv")):
        baseline = load_csv(os.path.join(args.baseline, "trajectory.csv"))
        baseline_rate, _ = load_analysis_params(args.baseline, args.config)

    metrics, switch_times, t_grid, vel, acc, jerk = compute_metrics(
        data, baseline, rate=rate, window=window, baseline_rate=baseline_rate)
    with open(os.path.join(args.data, "metrics.json"), "w") as f:
        json.dump(metrics, f, ensure_ascii=False, indent=2)
    write_markdown(metrics, os.path.join(args.data, "metrics.md"))

    plot_joints(data, switch_times, os.path.join(args.data, "joints.png"), window)
    plot_velocity(data, switch_times, os.path.join(args.data, "velocity.png"), window)
    plot_jerk(t_grid, switch_times, jerk, os.path.join(args.data, "jerk.png"),
              window, rate)
    plot_zmp(data, switch_times, os.path.join(args.data, "zmp.png"), window)
    plot_feet(data, os.path.join(args.data, "feet.png"))
    plot_timeline(data, switch_times, os.path.join(args.data, "gait_timeline.png"))
    if baseline is not None:
        plot_comparison(metrics, os.path.join(args.data, "comparison.png"))
    render_gif(data, model, os.path.join(args.data, "walking.gif"))

    print(json.dumps(metrics, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
