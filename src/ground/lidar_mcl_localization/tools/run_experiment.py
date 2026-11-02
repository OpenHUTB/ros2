# -*- coding: utf-8 -*-
"""完整实验：采集数据 → 训练神经网络 → 三种运动模型对比 → 出图。

为什么这套实验跑在离线仿真里（而不是 Gazebo）
-------------------------------------------
Gazebo 用来验证"ROS 链路是通的"（10 Hz 出位姿、能收敛，见文档 §6.1），
但它每次运行的接触求解有细微差异、且单次要几十秒，不方便做几十组重复实验。
本脚本用**地图射线投射**合成激光（同一张地图、同一台机器人），
好处是：确定性、可复现、几秒跑完，适合做定量对比。

两者用的是**同一份算法代码**（`particle_filter` / `sensor_model` / `motion_model`），
区别只在"激光从哪来"，所以结论可以互相印证。

用法（在包根目录下）：
    python3 tools/run_experiment.py                     # 默认参数
    python3 tools/run_experiment.py --episodes 60 --epochs 800
"""

import argparse
import csv
import math
import os
import sys

import numpy as np

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PKG_ROOT, "src"))
sys.path.insert(0, os.path.join(PKG_ROOT, "config"))

from lidar_mcl_localization.geometry import compose, pose_error          # noqa: E402
from lidar_mcl_localization.map_model import OccupancyMap                # noqa: E402
from lidar_mcl_localization.motion_model import MotionModel, odometry_delta  # noqa: E402
from lidar_mcl_localization.nn_motion import (MotionMLP, NNMotionCorrector,  # noqa: E402
                                              StandardScaler, correction_from_output,
                                              features_from_delta)
from lidar_mcl_localization.particle_filter import ParticleFilter        # noqa: E402
from lidar_mcl_localization.sensor_model import LikelihoodFieldModel     # noqa: E402
from scene_layout import LIDAR_SAMPLES, SPAWN                            # noqa: E402

MAX_RANGE = 8.0
DT = 0.1


# --------------------------------------------------------------------- 工具
def make_scan(occ, pose, angles, rng, noise=0.01):
    ranges = occ.raycast(pose, angles, MAX_RANGE)
    ranges = ranges + rng.normal(0.0, noise, ranges.size)
    return np.clip(ranges, 0.12, MAX_RANGE)


def slip_law(v, w):
    """轮胎滑移率：随转向率单调增长（与文档 §6.2 的实测一致）。

    物理含义：转向时轮胎既滚又刮，实际位移小于轮子转角对应的位移，
    而编码器只数轮子转了多少 —— 所以**里程计会高估走过的距离与转过的角度**。
    这正好是"常数标定系数修不掉、需要工况相关修正"的那部分误差。
    """
    return 0.020 + 0.100 * min(abs(w) / 1.20, 1.0)


def odom_step(odom, delta, rng, v, w, noise_s=0.010, noise_th=0.004,
              scale_s=1.020, scale_th=1.010):
    """模拟里程计：标定误差 + 工况相关的滑移 + 随机噪声。"""
    ds, dth = delta
    slip = slip_law(v, w)
    ds_od = ds * scale_s * (1.0 + slip) * (1.0 + rng.normal(0, noise_s))
    dth_od = dth * scale_th * (1.0 + 0.6 * slip) + rng.normal(0, noise_th)
    return compose(odom, (ds_od, dth_od)), (ds_od, dth_od)


def collect(occ, angles, episodes=40, steps=20, seed=1):
    """按"一段固定 (v, ω)"采样，标签用**窗口聚合**的比值。

    为什么不逐帧算比值：单帧的 dθ_odom 可能接近 0，比值会爆到几十甚至负号乱跳，
    网络学到的全是噪声（实测 val MSE 卡在 0.23 下不来）。
    把 20 帧的 Δ 累加后再取比值，标签就是平滑的 k_s(v,ω)、k_th(v,ω)。
    """
    rng = np.random.RandomState(seed)
    X, Y = [], []
    for _ in range(episodes):
        pose = (SPAWN["x"] + rng.uniform(-1.0, 1.0),
                SPAWN["y"] + rng.uniform(-0.5, 0.5),
                SPAWN["yaw"] + rng.uniform(-0.3, 0.3))
        # 覆盖三类工况：纯直行(w=0)、纯旋转(v=0)、边跑边转。
        # 少了任何一类，网络在该工况上就是外推 —— 实测少了纯旋转，
        # k_th 会被截断在 0.80（真值需要 0.94），修正反而变差。
        if rng.uniform() < 0.25:
            v = 0.0
        else:
            v = float(rng.uniform(0.10, 0.60))
        if rng.uniform() < 0.30:
            w = 0.0
        else:
            w = float(np.sign(rng.uniform(-1, 1)) * rng.uniform(0.05, 1.20))

        ds_true_sum = ds_od_sum = dth_true_sum = dth_od_sum = 0.0
        abs_dth_od = 0.0
        for _k in range(steps):
            delta = (v * DT, w * DT)
            pose = compose(pose, delta)
            ds_true_sum += delta[0]
            dth_true_sum += delta[1]
            slip = slip_law(v, w)
            ds_od = delta[0] * 1.020 * (1.0 + slip)
            dth_od = delta[1] * 1.010 * (1.0 + 0.6 * slip)
            ds_od_sum += ds_od
            dth_od_sum += dth_od
            abs_dth_od += abs(dth_od)

        # 只有"既没走也没转"才是无效样本。纯旋转时 ds_od≈0，
        # 若按 ds 过滤会把这类工况整段丢掉 —— 实测丢掉后网络在纯旋转上外推，
        # k_th 被截断到 0.80（真值需要 0.94）。
        if ds_od_sum < 1e-3 and abs(dth_od_sum) < 1e-3:
            continue
        k_s = (ds_true_sum / ds_od_sum) if ds_od_sum > 1e-3 else 1.0
        k_th = (dth_true_sum / dth_od_sum) if abs(dth_od_sum) > 0.02 else 1.0
        noise = 1.0 + 2.5 * slip_law(v, w) / 0.12
        X.append(features_from_delta(ds_od_sum / steps, dth_od_sum / steps, dt=DT))
        Y.append([k_s, k_th, min(noise, 4.0)])
    return np.array(X), np.array(Y)

def train(X, Y, epochs=600, lr=1e-2, seed=0):
    scaler = StandardScaler().fit(X)
    xs = scaler.transform(X)
    net = MotionMLP(seed=seed)
    rng = np.random.RandomState(seed)
    idx = rng.permutation(len(xs))
    n_val = max(1, int(0.2 * len(xs)))
    val, tr = idx[:n_val], idx[n_val:]

    hist = []
    for ep in range(epochs):
        p = rng.permutation(tr)
        for i in range(0, len(p), 64):
            b = p[i:i + 64]
            loss = net.backward(xs[b], Y[b], lr=lr)
        if ep % 100 == 0 or ep == epochs - 1:
            v = float(np.mean((net.predict(xs[val]) - Y[val]) ** 2))
            hist.append((ep, v))
    return net, scaler, hist


# ------------------------------------------------------------------- 评估
BENCH = [(0.40, 0.0, 6.0), (0.0, 0.80, 2.0)] * 4      # 方形循环


def run_benchmark(occ, angles, mode, net=None, scaler=None, num_particles=600,
                  seed=11, alpha=(0.12, 0.03, 0.12, 0.03), scan_every=1):
    """跑一遍基准轨迹，返回逐帧误差曲线。

    mode: 'odom'     只用里程计
          'odom_nn'  只用里程计 + 神经网络修正（不使用激光，专门衡量网络对里程计本身的价值）
          'mcl'      标准粒子滤波
          'nn'       粒子滤波 + 神经网络修正运动模型
    """
    rng = np.random.RandomState(seed)
    plan = []
    for v, w, dur in BENCH:
        plan.extend([(v, w)] * int(round(dur / DT)))

    truth = (SPAWN["x"], SPAWN["y"], SPAWN["yaw"])
    truth0 = truth
    odom = (0.0, 0.0, 0.0)
    odom_nn = (0.0, 0.0, 0.0)

    pf = None
    corrector = None
    if mode in ("mcl", "nn"):
        pf = ParticleFilter(num_particles=num_particles, init_pose=truth,
                            init_std=(0.20, 0.20, 0.10), rng=rng)
    if mode in ("nn", "odom_nn"):
        corrector = NNMotionCorrector(net, scaler)

    motion = MotionModel(alpha=alpha)
    sensor = LikelihoodFieldModel(sigma=0.20, beam_step=8)

    def to_odom_frame(pose):
        dx, dy = pose[0] - truth0[0], pose[1] - truth0[1]
        c, s = math.cos(-truth0[2]), math.sin(-truth0[2])
        return (c * dx - s * dy, s * dx + c * dy,
                (pose[2] - truth0[2] + math.pi) % (2 * math.pi) - math.pi)

    rows = []
    for k, (v, w) in enumerate(plan):
        delta = (v * DT, w * DT)
        truth = compose(truth, delta)
        odom, od_inc = odom_step(odom, delta, rng, v, w)
        ranges = make_scan(occ, truth, angles, rng)
        odom_delta = odometry_delta((0.0, 0.0, 0.0), (v * DT, 0.0, w * DT))

        if mode == "odom":
            est = odom
        elif mode == "odom_nn":
            # 纯里程计 + 网络修正：完全不看激光，专门衡量网络本身的价值
            corrector.push(od_inc[0], od_inc[1])
            ds_fix, dth_fix, _noise = corrector(od_inc)
            odom_nn = compose(odom_nn, (ds_fix, dth_fix))
            est = odom_nn
        else:
            if corrector is not None:
                corrector.push(odom_delta[0], odom_delta[1])
                pf.predict(motion, odom_delta, corrector=corrector,
                           context=odom_delta)
            else:
                pf.predict(motion, odom_delta)
            # 观测更新可以按 scan_every 抽稀：模拟激光频率低或计算资源紧张的情形，
            # 此时里程计质量对定位的影响会显著变大（见文档 §6.3 的频率扫描）
            if k % scan_every == 0:
                a_v, r_v = sensor.subsample(angles, ranges, MAX_RANGE)
                pf.update(sensor, a_v, r_v, occ, MAX_RANGE)
                pf.maybe_resample(0.5)
            est = pf.estimate()

        e_pos, e_yaw = pose_error(est, truth if mode not in ("odom", "odom_nn")
                                  else to_odom_frame(truth))
        rows.append(dict(step=k, t=k * DT, x=truth[0], y=truth[1], yaw=truth[2],
                         est_x=est[0], est_y=est[1], est_yaw=est[2],
                         err_pos=e_pos, err_yaw=math.degrees(e_yaw)))
    return rows


# ------------------------------------------------------------------- 输出
def write_csv(path, rows):
    with open(path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        for r in rows:
            writer.writerow(r)


def summarise(rows):
    e = np.array([r["err_pos"] for r in rows])
    y = np.array([r["err_yaw"] for r in rows])
    return dict(pos_mean=float(e.mean()), pos_max=float(e.max()),
                yaw_mean=float(y.mean()), yaw_max=float(y.max()),
                pos_final=float(e[-1]), yaw_final=float(y[-1]))


def make_plots(out_dir, occ, angles, results, hist):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:                                  # noqa: BLE001
        print("[图] 跳过（matplotlib 不可用：%s）" % exc)
        return []

    made = []

    # 1) 误差曲线
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.6))
    labels = {"① 纯里程计": "odometry only",
          "② 里程计 + 神经网络": "odometry + NN",
          "③ MCL（不用网络）": "MCL",
          "④ MCL + 神经网络": "MCL + NN"}
    for name, rows in results.items():
        t = [r["t"] for r in rows]
        axes[0].plot(t, [r["err_pos"] for r in rows], label=labels.get(name, name))
        axes[1].plot(t, [r["err_yaw"] for r in rows], label=labels.get(name, name))
    axes[0].set_xlabel("time [s]"); axes[0].set_ylabel("position error [m]")
    axes[0].set_title("position error [m]"); axes[0].grid(alpha=0.3); axes[0].legend()
    axes[1].set_xlabel("time [s]"); axes[1].set_ylabel("yaw error [deg]")
    axes[1].set_title("heading error [deg]"); axes[1].grid(alpha=0.3); axes[1].legend()
    fig.tight_layout()
    p = os.path.join(out_dir, "error_comparison.png")
    fig.savefig(p, dpi=130); plt.close(fig); made.append(p)

    # 2) 轨迹对比（地图底图）
    fig, ax = plt.subplots(figsize=(6.4, 5.2))
    free = (occ.grid == 254)
    ax.imshow(np.where(occ.occupied, 0, np.where(free, 1, 0.6)),
              cmap="gray", origin="upper",
              extent=[occ.origin[0], occ.origin[0] + occ.width * occ.resolution,
                      occ.origin[1], occ.origin[1] + occ.height * occ.resolution])
    for name, rows in results.items():
        ax.plot([r["est_x"] for r in rows], [r["est_y"] for r in rows],
                linewidth=1.4, label=labels.get(name, name))
    truth_rows = list(results.values())[0]
    ax.plot([r["x"] for r in truth_rows], [r["y"] for r in truth_rows],
            "k--", linewidth=1.0, label="ground truth")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("trajectories on occupancy map (4 laps)")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    p = os.path.join(out_dir, "trajectories.png")
    fig.savefig(p, dpi=130); plt.close(fig); made.append(p)

    # 3) 训练曲线
    if hist:
        fig, ax = plt.subplots(figsize=(5.2, 3.2))
        ax.semilogy([h[0] for h in hist], [h[1] for h in hist], "o-")
        ax.set_xlabel("epoch"); ax.set_ylabel("val MSE")
        ax.set_title("neural motion model: validation MSE")
        ax.grid(alpha=0.3, which="both")
        fig.tight_layout()
        p = os.path.join(out_dir, "nn_training.png")
        fig.savefig(p, dpi=130); plt.close(fig); made.append(p)

    return made


def main(argv=None):
    ap = argparse.ArgumentParser(description="MCL 三组对比实验")
    ap.add_argument("--episodes", type=int, default=40)
    ap.add_argument("--epochs", type=int, default=600)
    ap.add_argument("--particles", type=int, default=600)
    ap.add_argument("--out", default=os.path.join(PKG_ROOT, "results"))
    args = ap.parse_args(argv)

    os.makedirs(args.out, exist_ok=True)
    occ = OccupancyMap.from_files(os.path.join(PKG_ROOT, "maps", "mcl_room.pgm"))
    angles = np.linspace(-math.pi, math.pi, LIDAR_SAMPLES, endpoint=False)
    print("[地图] %s" % occ)

    print("\n① 采集训练数据（%d 段随机工况）" % args.episodes)
    X, Y = collect(occ, angles, episodes=args.episodes)
    print("   样本 %d 条，标签均值 [k_s %.4f, k_th %.4f, noise %.3f]"
          % (len(X), Y[:, 0].mean(), Y[:, 1].mean(), Y[:, 2].mean()))

    print("\n② 训练神经网络（%d 轮）" % args.epochs)
    net, scaler, hist = train(X, Y, epochs=args.epochs)
    for ep, v in hist:
        print("   epoch %4d  val MSE %.6f" % (ep, v))

    print("\n③ 基准轨迹对比")
    results = {}
    for mode, label in (("odom", "① 纯里程计"),
                     ("odom_nn", "② 里程计 + 神经网络"),
                     ("mcl", "③ MCL（不用网络）"),
                     ("nn", "④ MCL + 神经网络")):
        rows = run_benchmark(occ, angles, mode, net, scaler, num_particles=args.particles)
        results[label] = rows
        s = summarise(rows)
        print("   %-16s 位置 均值 %.3f m / 最大 %.3f m   航向 均值 %5.2f° / 最大 %6.2f°"
              % (label, s["pos_mean"], s["pos_max"], s["yaw_mean"], s["yaw_max"]))
        write_csv(os.path.join(args.out, "trace_%s.csv" % mode), rows)

    print("\n④ 出图")
    made = make_plots(args.out, occ, angles, results, hist)
    for p in made:
        print("[图] %s" % p)

    print("\n[完成] 结果目录：%s" % args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
