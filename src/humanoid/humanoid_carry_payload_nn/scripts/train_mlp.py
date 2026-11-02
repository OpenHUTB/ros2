#!/usr/bin/env python3
"""train_mlp.py —— 行为克隆（Behaviour Cloning）训练平衡神经网络。

流程：
  1. 用 PD 专家在 0/2/4 kg 三种负载、若干速度/转向指令下跑仿真，记录
     "状态特征 -> 专家补偿角" 的数据对；
  2. 用这些数据对训练 9→32→32→3 的全连接网络（tanh 激活，Adam 优化）；
  3. 保存权重到 models/balance_mlp.npz（+ 同名 .json 记录特征名与结构）。

用法（在包目录或任意目录均可）：
    python3 scripts/train_mlp.py                 # 默认 0/2/4 kg，20 s/场景
    python3 scripts/train_mlp.py --epochs 80 --seconds 30
"""

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from model import FEATURE_NAMES, OUTPUT_NAMES, BalanceMLP, PDExpert  # noqa: E402
from sim_core import CarryPayloadSim, package_path                 # noqa: E402

DEFAULT_MASSES = (0.0, 2.0, 4.0)


def command_profile(t, rng):
    """随时间变化的指令：巡航速度 + 偶尔加减速/转向，覆盖不同运动状态。"""
    base_v = rng.choice([0.0, 0.12, 0.20, 0.28])
    v = base_v * (1.0 + 0.15 * np.sin(0.4 * t))
    if 3.0 < t < 5.0:
        v *= 0.3                       # 减速段
    w = 0.25 * np.sin(0.25 * t) if t > 2.0 else 0.0
    return float(np.clip(v, -0.3, 0.3)), float(w)


def collect(args):
    """用 PD 专家采集 (特征, 补偿角) 数据对。"""
    rng = np.random.default_rng(args.seed)
    expert = PDExpert()
    xs, ys = [], []
    for mass in args.masses:
        sim = CarryPayloadSim(payload_mass=mass)
        sim.reset(mass, phase=float(rng.random()))
        steps = int(args.seconds / sim.dt)
        for i in range(steps):
            t = i * sim.dt
            v_cmd, w_cmd = command_profile(t, rng)
            feats = sim.step(v_cmd=v_cmd, w_cmd=w_cmd, controller=expert)
            xs.append([feats[name] for name in FEATURE_NAMES])
            ys.append([feats["expert"][name] for name in OUTPUT_NAMES])
        print("[collect] 负载 %4.1f kg：累计样本 %d" % (mass, len(xs)))
        sim.close()
    return np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)


def forward(weights, x):
    """前向推理（最后一层不加激活），返回各层激活以便反向传播。"""
    acts = [x]
    n = len(weights) // 2
    h = x
    for i in range(n):
        h = h @ weights[2 * i] + weights[2 * i + 1]
        if i < n - 1:
            h = np.tanh(h)
        acts.append(h)
    return h, acts


def backward(weights, acts, y_true, y_pred):
    """MSE 损失对权重的梯度。"""
    n = len(weights) // 2
    grads = [None] * len(weights)
    delta = 2.0 * (y_pred - y_true) / y_true.shape[0]
    for i in range(n - 1, -1, -1):
        grads[2 * i] = acts[i].T @ delta
        grads[2 * i + 1] = delta.sum(axis=0)
        if i > 0:
            delta = (delta @ weights[2 * i].T) * (1.0 - acts[i] ** 2)
    return grads


def train(args):
    t0 = time.time()
    x, y = collect(args)
    mean, std = x.mean(axis=0), x.std(axis=0)
    std[std < 1e-6] = 1.0
    xn = (x - mean) / std

    n_val = max(1, int(0.15 * len(xn)))
    idx = np.random.default_rng(args.seed).permutation(len(xn))
    val_idx, train_idx = idx[:n_val], idx[n_val:]

    rng = np.random.default_rng(args.seed + 1)
    sizes = [len(FEATURE_NAMES)] + list(args.hidden) + [len(OUTPUT_NAMES)]
    weights = []
    for i in range(len(sizes) - 1):
        limit = np.sqrt(6.0 / (sizes[i] + sizes[i + 1]))
        weights.append(rng.uniform(-limit, limit, (sizes[i], sizes[i + 1])))
        weights.append(np.zeros(sizes[i + 1]))

    m = [np.zeros_like(w) for w in weights]
    v = [np.zeros_like(w) for w in weights]
    beta1, beta2, eps = 0.9, 0.999, 1e-8
    batch = args.batch

    for epoch in range(args.epochs):
        perm = np.random.permutation(train_idx)
        losses = []
        for start in range(0, len(perm), batch):
            b = perm[start:start + batch]
            y_pred, acts = forward(weights, xn[b])
            losses.append(float(np.mean((y_pred - y[b]) ** 2)))
            grads = backward(weights, acts, y[b], y_pred)
            step = epoch * (len(perm) // batch + 1) + start // batch + 1
            for k in range(len(weights)):
                m[k] = beta1 * m[k] + (1 - beta1) * grads[k]
                v[k] = beta2 * v[k] + (1 - beta2) * grads[k] ** 2
                mhat = m[k] / (1 - beta1 ** step)
                vhat = v[k] / (1 - beta2 ** step)
                weights[k] -= args.lr * mhat / (np.sqrt(vhat) + eps)
        if (epoch + 1) % max(1, args.epochs // 10) == 0:
            y_val, _ = forward(weights, xn[val_idx])
            val_mse = float(np.mean((y_val - y[val_idx]) ** 2))
            print("[train] epoch %3d  train_mse=%.4f  val_mse=%.4f"
                  % (epoch + 1, np.mean(losses), val_mse))

    net = BalanceMLP(weights=weights, hidden=tuple(args.hidden),
                     corr_limit=args.corr_limit, mean=mean, std=std)
    out = args.output
    net.save(out)
    print("[train] 样本 %d，训练用时 %.1f s" % (len(x), time.time() - t0))
    print("[train] 权重已保存：%s（同名 .json 为元信息）" % out)


def main():
    ap = argparse.ArgumentParser(description="行为克隆训练平衡神经网络")
    ap.add_argument("--masses", type=float, nargs="+", default=list(DEFAULT_MASSES))
    ap.add_argument("--seconds", type=float, default=20.0, help="每个负载场景的采集时长")
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--batch", type=int, default=256)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--hidden", type=int, nargs="+", default=[32, 32])
    ap.add_argument("--corr_limit", type=float, default=0.30)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--output", default=package_path("models/balance_mlp.npz"))
    train(ap.parse_args())


if __name__ == "__main__":
    main()
