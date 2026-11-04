#!/usr/bin/env python3
"""train_mlp.py —— 行为克隆（Behaviour Cloning）训练平衡神经网络。

流程：
  1. 用 PD 专家在 0/2/4 kg 三种负载、若干速度/转向指令下跑仿真，记录
     "状态特征 -> 专家补偿角" 的数据对；
  2. 用这些数据对训练 9→32→32→3 的全连接网络（tanh 激活，Adam 优化）；
  3. 保存权重到 models/balance_mlp.npz（+ 同名 .json 记录特征名与结构）。

用法（在包目录或任意目录均可）：
    python3 scripts/train_mlp.py                 # 默认 0/2/4 kg，20 s/场景（约 3.0 万条样本）
    python3 scripts/train_mlp.py --epochs 80 --seconds 30
    python3 scripts/train_mlp.py --seconds 25    # 3.75 万条样本（文档中的口径）
"""

import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from model import FEATURE_NAMES, OUTPUT_NAMES, BalanceMLP, PDExpert  # noqa: E402
from sim_core import CarryPayloadSim, package_path                 # noqa: E402

DEFAULT_MASSES = (0.0, 2.0, 4.0)
#: 每个"巡航片段"的时长（s）。基础速度按片段固定，片段边界才重新采样，
#: 否则 500 Hz 循环里每步换一次速度，采到的分布不是"按某个速度巡航"。
SEGMENT_SECONDS = 4.0
#: 基础巡航速度候选（m/s）
BASE_SPEEDS = (0.0, 0.12, 0.20, 0.28)
#: 转向幅度候选（rad/s）
BASE_YAW_RATES = (0.0, 0.15, 0.25)


def command_profile(t, base_v, base_w):
    """在给定基础速度/转向下的平滑指令（加减速段 + 正弦转向）。"""
    v = base_v * (1.0 + 0.15 * np.sin(0.4 * t))
    if 3.0 < t < 5.0:
        v *= 0.3                       # 减速段
    w = base_w * np.sin(0.25 * t) if t > 2.0 else 0.0
    return float(np.clip(v, -0.3, 0.3)), float(w)


def collect(args, rng):
    """用 PD 专家采集 (特征, 补偿角) 数据对。

    返回 (x, y, groups)：groups 为片段编号，训练/验证按片段切分，
    避免时间上相邻的样本同时出现在两边。
    """
    expert = PDExpert()
    xs, ys, groups = [], [], []
    gid = 0
    for mass in args.masses:
        sim = CarryPayloadSim(payload_mass=mass)
        sim.reset(mass, phase=float(rng.random()))
        steps = int(args.seconds / sim.dt)
        seg_len = max(1, int(SEGMENT_SECONDS / sim.dt))
        base_v = float(rng.choice(BASE_SPEEDS))
        base_w = float(rng.choice(BASE_YAW_RATES))
        mark = len(xs)                 # 当前片段的样本起点，跌倒时整段回滚
        resets = 0
        for i in range(steps):
            t = i * sim.dt
            if i > 0 and i % seg_len == 0:
                base_v = float(rng.choice(BASE_SPEEDS))
                base_w = float(rng.choice(BASE_YAW_RATES))
                gid += 1
                mark = len(xs)
            v_cmd, w_cmd = command_profile(t, base_v, base_w)
            feats = sim.step(v_cmd=v_cmd, w_cmd=w_cmd, controller=expert)
            if not sim.standing():
                # 摔倒后继续记录会把倒地姿态当成示教数据：整段丢弃并重置
                del xs[mark:], ys[mark:], groups[mark:]
                resets += 1
                sim.reset(mass, phase=float(rng.random()))
                gid += 1
                mark = len(xs)
                continue
            xs.append([feats[name] for name in FEATURE_NAMES])
            ys.append([feats["expert"][name] for name in OUTPUT_NAMES])
            groups.append(gid)
        print("[collect] 负载 %4.1f kg：累计样本 %d，跌倒重置 %d 次" % (mass, len(xs), resets))
        sim.close()
    return (np.asarray(xs, dtype=float), np.asarray(ys, dtype=float),
            np.asarray(groups, dtype=int))


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
    rng = np.random.default_rng(args.seed)
    x, y, groups = collect(args, rng)

    # 按片段切分：每 5 段取 1 段做验证，避免时间上相邻的样本同时落在两边
    uniq = np.unique(groups)
    val_groups = set(uniq[::5].tolist())
    val_mask = np.array([g in val_groups for g in groups])
    if val_mask.all() or not val_mask.any():
        idx = rng.permutation(len(x))
        n_val = max(1, int(0.15 * len(x)))
        val_mask = np.zeros(len(x), dtype=bool)
        val_mask[idx[:n_val]] = True

    # 归一化统计只用训练集，避免验证集信息泄漏
    mean = x[~val_mask].mean(axis=0)
    std = x[~val_mask].std(axis=0)
    std[std < 1e-6] = 1.0
    xn = (x - mean) / std

    val_idx = np.flatnonzero(val_mask)
    train_idx = np.flatnonzero(~val_mask)

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

    history = []
    for epoch in range(args.epochs):
        perm = rng.permutation(train_idx)
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
            history.append({"epoch": epoch + 1,
                            "train_mse": float(np.mean(losses)),
                            "val_mse": val_mse})
            print("[train] epoch %3d  train_mse=%.4f  val_mse=%.4f"
                  % (epoch + 1, np.mean(losses), val_mse))

    net = BalanceMLP(weights=weights, hidden=tuple(args.hidden),
                     corr_limit=args.corr_limit, mean=mean, std=std)
    out = args.output
    net.save(out)

    # 训练指标留档，便于复核文档中的 MSE 结论
    y_tr, _ = forward(weights, xn[train_idx])
    y_va, _ = forward(weights, xn[val_idx])
    metrics = {
        "samples": int(len(x)),
        "train_samples": int(len(train_idx)),
        "val_samples": int(len(val_idx)),
        "segments": int(len(uniq)),
        "val_segments": int(len(val_groups)),
        "seconds_per_scene": float(args.seconds),
        "masses": list(map(float, args.masses)),
        "epochs": int(args.epochs),
        "batch": int(args.batch),
        "lr": float(args.lr),
        "hidden": list(map(int, args.hidden)),
        "seed": int(args.seed),
        "corr_limit": float(args.corr_limit),
        "final_train_mse": float(np.mean((y_tr - y[train_idx]) ** 2)),
        "final_val_mse": float(np.mean((y_va - y[val_idx]) ** 2)),
        "history": history,
    }
    metrics_path = os.path.splitext(out)[0] + "_metrics.json"
    with open(metrics_path, "w", encoding="utf-8") as f:
        json.dump(metrics, f, ensure_ascii=False, indent=2)
    print("[train] 样本 %d（训练 %d / 验证 %d），最终 train_mse=%.2e  val_mse=%.2e"
          % (len(x), len(train_idx), len(val_idx),
             metrics["final_train_mse"], metrics["final_val_mse"]))
    print("[train] 训练用时 %.1f s" % (time.time() - t0))
    print("[train] 权重已保存：%s（同名 .json 为元信息，_metrics.json 为指标留档）" % out)


def main():
    ap = argparse.ArgumentParser(description="行为克隆训练平衡神经网络")
    ap.add_argument("--masses", type=float, nargs="+", default=list(DEFAULT_MASSES))
    ap.add_argument("--seconds", type=float, default=20.0, help="每个负载场景的采集时长")
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--batch", type=int, default=256)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--hidden", type=int, nargs="+", default=[32, 32])
    ap.add_argument("--corr_limit", type=float, default=0.50)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--output", default=package_path("models/balance_mlp.npz"))
    train(ap.parse_args())


if __name__ == "__main__":
    main()
