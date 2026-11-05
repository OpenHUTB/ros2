#!/usr/bin/env python3
"""离线仿真入口：对比航位推算与 EKF 融合定位，并输出图表。

用法示例：
    python3 sim/run_sim.py --traj eight
    python3 sim/run_sim.py --traj circle --pos-rate 5 --seed 7
    python3 sim/run_sim.py --traj s_curve --out results/s_curve.png
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from ros2_ekf_localization.ekf import EKFConfig, EKFLocalizer  # noqa: E402
from ros2_ekf_localization.sensors import (  # noqa: E402
    ImuNoise,
    OdometryNoise,
    PositionSensor,
    integrate_odometry,
)
from ros2_ekf_localization.trajectories import TRAJECTORY_BUILDERS  # noqa: E402

MODES = ("odom", "ekf_imu", "ekf_full")


def precompute_noise(traj, seed: int, pos_rate: float, pos_sigma: float = 0.15):
    """一次性生成全部传感器读数，保证三种模式面对完全相同的噪声。"""
    rng = np.random.default_rng(seed)
    odom = OdometryNoise()
    imu = ImuNoise()
    pos = PositionSensor(sigma=pos_sigma, rate_hz=pos_rate)

    n = len(traj)
    v_meas = np.empty(n)
    w_meas = np.empty(n)
    yaw_meas = np.empty(n)
    pos_avail = np.zeros(n, dtype=bool)
    pos_meas = np.zeros((n, 2))

    for k in range(n):
        v_meas[k], w_meas[k] = odom.corrupt(traj.v[k], traj.w[k], rng)
        yaw_meas[k] = imu.observe(traj.theta[k], rng)
        if pos.available(traj.t[k]):
            pos_avail[k] = True
            pos_meas[k] = pos.observe(traj.x[k], traj.y[k], rng)

    return v_meas, w_meas, yaw_meas, pos_avail, pos_meas


def run(mode: str, traj, noise, cfg: EKFConfig):
    """跑一种定位模式，返回 (位姿序列, 位置误差, 航向误差, NEES 或 None)。"""
    v_meas, w_meas, yaw_meas, pos_avail, pos_meas = noise
    dt = traj.dt
    n = len(traj)

    if mode == "odom":
        poses = integrate_odometry(v_meas, w_meas, dt, traj.state(0))
        nees = np.full(n, np.nan)
    else:
        ekf = EKFLocalizer(cfg, x0=traj.state(0), P0=np.eye(3) * 1e-3)
        poses = np.zeros((n, 3))
        nees = np.zeros(n)
        for k in range(n):
            poses[k] = ekf.x
            nees[k] = ekf.nees(traj.state(k))
            ekf.predict(v_meas[k], w_meas[k], dt)
            ekf.update_yaw(yaw_meas[k])
            if mode == "ekf_full" and pos_avail[k]:
                ekf.update_position(pos_meas[k, 0], pos_meas[k, 1])

    truth = np.column_stack([traj.x, traj.y, traj.theta])
    pos_err = np.hypot(poses[:, 0] - truth[:, 0], poses[:, 1] - truth[:, 1])
    head_err = np.abs(np.array([
        (lambda a: np.arctan2(np.sin(a), np.cos(a)))(poses[k, 2] - truth[k, 2])
        for k in range(n)
    ]))
    return poses, pos_err, head_err, nees


def summarize(mode: str, pos_err, head_err, nees) -> dict:
    nees_finite = nees[np.isfinite(nees)]
    return {
        "mode": mode,
        "final_pos_err": float(pos_err[-1]),
        "max_pos_err": float(np.max(pos_err)),
        "rms_pos_err": float(np.sqrt(np.mean(pos_err**2))),
        "final_head_err": float(head_err[-1]),
        "mean_nees": float(np.mean(nees_finite)) if nees_finite.size else float("nan"),
    }


def plot(traj_name: str, results: dict, truth, traj, pos_rate: float, out: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(16.5, 4.8))
    colors = {"odom": "#d62728", "ekf_imu": "#ff7f0e", "ekf_full": "#2ca02c"}
    labels = {
        "odom": "dead reckoning (odom only)",
        "ekf_imu": "EKF (odom + IMU yaw)",
        "ekf_full": "EKF (odom + IMU yaw + position)",
    }

    ax = axes[0]
    ax.plot(truth[:, 0], truth[:, 1], "k--", lw=1.6, label="ground truth")
    for mode in MODES:
        poses = results[mode]["poses"]
        ax.plot(poses[:, 0], poses[:, 1], color=colors[mode], lw=1.5, label=labels[mode])
    ax.plot(truth[0, 0], truth[0, 1], "o", color="#1f77b4", ms=9, label="start")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"Trajectory - {traj_name}")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.3)

    axes[1].set_title("Position error vs time")
    for mode in MODES:
        axes[1].plot(traj.t, results[mode]["pos_err"], color=colors[mode], lw=1.5,
                     label=f"{mode} (final {results[mode]['pos_err'][-1]:.2f} m)")
    axes[1].set_xlabel("t (s)")
    axes[1].set_ylabel("position error (m)")
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.3)

    axes[2].set_title("NEES (3-DOF, ideal mean = 3)")
    for mode in ("ekf_imu", "ekf_full"):
        nees = results[mode]["nees"]
        axes[2].plot(traj.t, nees, color=colors[mode], lw=1.2, alpha=0.85, label=mode)
    axes[2].axhline(3.0, color="k", ls="--", lw=1.0, label="ideal mean (3)")
    axes[2].set_xlabel("t (s)")
    axes[2].set_ylabel("NEES")
    axes[2].set_ylim(0.0, 30.0)
    axes[2].legend(fontsize=8)
    axes[2].grid(alpha=0.3)

    fig.suptitle(f"EKF localization - {traj_name}, position update @ {pos_rate:g} Hz", fontsize=11)
    fig.tight_layout()
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    fig.savefig(out, dpi=140)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description="EKF 融合定位离线仿真")
    ap.add_argument("--traj", choices=sorted(TRAJECTORY_BUILDERS), default="eight")
    ap.add_argument("--pos-rate", type=float, default=1.0, help="位置观测频率 (Hz)，0 表示无")
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    traj = TRAJECTORY_BUILDERS[args.traj](speed=0.8, dt=0.05)
    cfg = EKFConfig()
    noise = precompute_noise(traj, args.seed, args.pos_rate)

    results = {}
    for mode in MODES:
        poses, pos_err, head_err, nees = run(mode, traj, noise, cfg)
        results[mode] = {
            "poses": poses,
            "pos_err": pos_err,
            "head_err": head_err,
            "nees": nees,
            "metrics": summarize(mode, pos_err, head_err, nees),
        }

    truth = np.column_stack([traj.x, traj.y, traj.theta])
    out = args.out or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "results", f"{args.traj}.png"
    )
    plot(args.traj, results, truth, traj, args.pos_rate, out)

    print(f"轨迹={args.traj}  位置观测={args.pos_rate:g} Hz  种子={args.seed}  步数={len(traj)}")
    print("-" * 64)
    print(f"{'模式':<10}{'末端误差(m)':>12}{'最大误差(m)':>12}{'RMS(m)':>10}{'末端航向(rad)':>14}{'NEES':>8}")
    for mode in MODES:
        m = results[mode]["metrics"]
        nees = m["mean_nees"]
        nees_s = "—" if not np.isfinite(nees) else f"{nees:.2f}"
        print(f"{mode:<10}{m['final_pos_err']:>12.3f}{m['max_pos_err']:>12.3f}"
              f"{m['rms_pos_err']:>10.3f}{m['final_head_err']:>14.4f}{nees_s:>8}")
    print(f"  图表已保存: {os.path.abspath(out)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
