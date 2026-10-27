#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""完整实验：参数标定 → 采集数据 → 训练滑移补偿网络 → 三组对比。

流程：
    ① 直行 + 旋转两个实验，辨识 (c_s, k_head, c_theta)          ← 线性部分
    ② 在 MuJoCo 里随机工况采样，记录"标定后残余误差"作为标签      ← 训练数据
    ③ 训练 MLP（输入轮速/IMU，输出补偿系数）                     ← 非线性部分
    ④ 同一条 4x4 m 方形路径，对比三种里程计的终点误差             ← 评价

用法（在包根目录下）：
    python3 tools/run_experiment.py                # 全流程
    python3 tools/run_experiment.py --episodes 40  # 改数据量
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

PKG_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG_ROOT / "src"))

from diff_drive_odom_calib import config                                    # noqa: E402
from diff_drive_odom_calib.calibration import OdometryIntegrator, identify  # noqa: E402
from diff_drive_odom_calib.mujoco_env import DiffDriveEnv                    # noqa: E402
from diff_drive_odom_calib.slip_nn import (SlipMLP, StandardScaler,          # noqa: E402
                                           compensate, features)

DT = 1.0 / config.CONTROL_HZ


# ----------------------------------------------------------------------
def run_segments(env, segments, warmup=0.0):
    """按 (v, w, 秒) 序列驱动，返回逐控制周期的记录。

    warmup：丢弃开头的若干秒。**标定必须丢**——起步时轮子从 0 加速到目标转速，
    前 1~2 秒滑移率高达 30~45%（实测），把这段算进 c_s 会让标定系数偏小
    （实测 c_s 从应有的 0.966 掉到 0.854），随后整条路径被系统性过补偿。
    """
    env.reset((0.0, 0.0, 0.0))
    log = []
    skip = int(round(warmup * config.CONTROL_HZ))
    k = 0
    for v, w, dur in segments:
        env.set_twist(v, w)
        for _ in range(int(round(dur * config.CONTROL_HZ))):
            st0 = env.state()
            t0 = env.wheel_angles()
            env.step()
            st1 = env.state()
            t1 = env.wheel_angles()
            ds_true = math.hypot(st1.x - st0.x, st1.y - st0.y)
            dth_true = (st1.yaw - st0.yaw + math.pi) % (2 * math.pi) - math.pi
            k += 1
            if k <= skip:
                continue
            log.append(dict(
                ds_true=ds_true, dth_true=dth_true,
                dphi_left=t1[0] - t0[0], dphi_right=t1[1] - t0[1],
                wl=st1.wheel[0], wr=st1.wheel[1],
                gyro=st1.w, acc=st1.v))
    return log


def calibrate(env):
    """直行 + 原地旋转两个实验，辨识三个线性系数。"""
    straight = run_segments(env,
                            [(config.STRAIGHT_SPEED, 0.0,
                              config.STRAIGHT_DISTANCE / config.STRAIGHT_SPEED)],
                            warmup=1.5)
    rot_t = config.ROTATION_TURNS * 2 * math.pi / config.ROTATION_SPEED
    rotation = run_segments(env, [(0.0, config.ROTATION_SPEED, rot_t)], warmup=1.5)
    params = identify(straight, rotation,
                      r_nominal=config.R_NOMINAL, b_nominal=config.B_NOMINAL)
    return params, straight, rotation


def collect_training(env, params, episodes=30, seed=0):
    """随机工况采样，标签是"标定后残余误差"的比例。"""
    rng = np.random.RandomState(seed)
    X, Y = [], []
    for _ in range(episodes):
        # 只在"有转向"的工况采样：直行段 dth_odom≈0，比值没有意义；
        # 而转向恰恰是滑移非线性最强的区间（实测 ω=1.2 时滑移 17%）。
        v = float(rng.uniform(0.10, 0.60))
        w = float(np.sign(rng.uniform(-1, 1)) * rng.uniform(0.30, 1.30))
        seg = [(v, w, 1.2)]
        log = run_segments(env, seg)
        integ = OdometryIntegrator(c_s=params["c_s"], c_theta=params["c_theta"],
                                   k_head=params["k_head"],
                                   r_nominal=config.R_NOMINAL, b_nominal=config.B_NOMINAL)
        for o in log:
            ds_od, dth_od = integ.step(o["dphi_left"], o["dphi_right"])
            # 分母太小会让比值爆炸，直接丢弃这些样本
            if abs(ds_od) < 2e-4 or abs(dth_od) < 2e-3:
                continue
            X.append(features(o["wl"], o["wr"], o["gyro"], o["acc"]))
            Y.append([o["ds_true"] / ds_od, o["dth_true"] / dth_od])
    return np.array(X), np.array(Y)


def train_net(X, Y, epochs=600, lr=2e-3, seed=0, verbose=True):
    rng = np.random.RandomState(seed)
    scaler = StandardScaler().fit(X)
    xs = scaler.transform(X)
    n = len(xs)
    idx = rng.permutation(n)
    n_val = max(1, int(0.2 * n))
    val, tr = idx[:n_val], idx[n_val:]
    net = SlipMLP(seed=seed)
    hist = []
    for ep in range(epochs):
        p = rng.permutation(tr)
        for i in range(0, len(p), 64):
            b = p[i:i + 64]
            loss = net.backward(xs[b], Y[b], lr=lr)
        if ep % 100 == 0 or ep == epochs - 1:
            v = float(np.mean((net.predict(xs[val]) - Y[val]) ** 2))
            hist.append((ep, v))
            if verbose:
                print("   epoch %3d  val MSE %.6f" % (ep, v))
    return net, scaler, hist


def evaluate(env, params, net=None, scaler=None, side=None):
    """方形路径，返回终点误差与轨迹。net 为 None 时只做标定。"""
    side = config.SQUARE_SIDE if side is None else side
    segs = []
    turn = (math.pi / 2) / config.ROTATION_SPEED
    for _ in range(4):
        segs.append((config.STRAIGHT_SPEED, 0.0, side / config.STRAIGHT_SPEED))
        segs.append((0.0, config.ROTATION_SPEED, turn))
    log = run_segments(env, segs)

    integ = OdometryIntegrator(c_s=params.get("c_s", 1.0),
                               c_theta=params.get("c_theta", 1.0),
                               k_head=params.get("k_head", 0.0),
                               r_nominal=config.R_NOMINAL, b_nominal=config.B_NOMINAL)
    ox = oy = oth = 0.0
    for o in log:
        ds_od, dth_od = integ.step(o["dphi_left"], o["dphi_right"])
        if net is not None:
            ds_od, dth_od = compensate(net, scaler, o["wl"], o["wr"], o["gyro"], o["acc"],
                                       ds_od, dth_od)
        ox += ds_od * math.cos(oth + 0.5 * dth_od)
        oy += ds_od * math.sin(oth + 0.5 * dth_od)
        oth = (oth + dth_od + math.pi) % (2 * math.pi) - math.pi
    st = env.state()
    err = math.hypot(ox - st.x, oy - st.y)
    return dict(err=err, dth=(oth - st.yaw + math.pi) % (2 * math.pi) - math.pi,
                path=sum(o["ds_true"] for o in log))


def main(argv=None):
    ap = argparse.ArgumentParser(description="标定 + 神经网络滑移补偿 三组对比")
    ap.add_argument("--episodes", type=int, default=30)
    ap.add_argument("--epochs", type=int, default=600)
    ap.add_argument("--scene", default=None)
    args = ap.parse_args(argv)

    env = DiffDriveEnv(config.resolve_scene_path(args.scene))

    print("\n① 参数标定（直行 2 m + 原地转 3 圈）")
    params, st, ro = calibrate(env)
    print("   c_s=%.5f  k_head=%+.5f rad/m  c_theta=%.5f"
          % (params["c_s"], params["k_head"], params["c_theta"]))

    print("\n② 采集训练数据（%d 段随机工况）" % args.episodes)
    X, Y = collect_training(env, params, episodes=args.episodes)
    print("   样本 %d 条，特征维度 %d，标签均值 [%.4f, %.4f]"
          % (len(X), X.shape[1], Y[:, 0].mean(), Y[:, 1].mean()))

    print("\n③ 训练 MLP")
    net, scaler, hist = train_net(X, Y, epochs=args.epochs)

    out_dir = config.DEFAULT_OUTPUT_DIR
    out_dir.mkdir(parents=True, exist_ok=True)
    net.save(str(out_dir / "slip_mlp.npz"))
    scaler.save(str(out_dir / "slip_scaler.npz"))

    print("\n④ 方形路径 %.0f×%.0f m 三组对比" % (config.SQUARE_SIDE, config.SQUARE_SIDE))
    raw = evaluate(env, {"c_s": 1.0, "c_theta": 1.0, "k_head": 0.0})
    cal = evaluate(env, params)
    both = evaluate(env, params, net, scaler)
    print("   %-22s%12s%14s" % ("方案", "终点误差", "航向误差"))
    print("   " + "-" * 48)
    for name, r in (("① 原始里程计", raw), ("② 参数标定", cal), ("③ 标定 + 神经网络", both)):
        print("   %-22s%10.3f m%12.2f°" % (name, r["err"], math.degrees(r["dth"])))
    print("   路径长度 %.1f m" % both["path"])

    metrics = dict(params=params, samples=int(len(X)),
                   raw=[raw["err"], math.degrees(raw["dth"])],
                   cal=[cal["err"], math.degrees(cal["dth"])],
                   cal_nn=[both["err"], math.degrees(both["dth"])],
                   path=both["path"],
                   train_hist=[[int(e), float(v)] for e, v in hist])
    with open(out_dir / "experiment.json", "w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)
    print("\n[导出] %s/slip_mlp.npz, experiment.json" % out_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
