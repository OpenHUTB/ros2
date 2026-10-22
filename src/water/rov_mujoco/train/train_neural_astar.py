#!/usr/bin/env python3
"""
任务 3 —— Neural A* 引导场网络训练
====================================
专家: 解析椭圆走廊先验  Phi = exp(-(d_start + d_goal - d_direct) / 2)
学生: 13 -> 64 -> 32 -> 1 MLP (tanh 隐藏层, sigmoid 输出)

输入特征 (逐栅格):
  [0] (x - x_goal) / diag
  [1] (y - y_goal) / diag
  [2] dist(start, cell) / diag
  [3] dist(goal, cell) / diag
  [4:13] 3x3 邻域占据 (膨胀后障碍图, 越界视为障碍)

训练完成后保存到 rov_mujoco/planner_weights.npz，
NeuralAStarPlanner 在 plan() 中以该网络前向输出替代解析公式。

运行:
    python3 train/train_neural_astar.py [--maps 160] [--epochs 24]
"""

import argparse
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from rov_mujoco.nn_core import Adam, NumpyMLP, save_npz  # noqa: E402
from rov_mujoco.neural_path_planner import guidance_features, analytic_guidance  # noqa: E402

FEATURE_DIM = 13


def random_map(rng, size=64):
    """生成带随机墙体/圆形障碍的二值地图（0 自由, 1 占据）"""
    grid = np.zeros((size, size), dtype=bool)
    # 2~4 个矩形墙体
    for _ in range(int(rng.integers(2, 5))):
        bw = int(rng.integers(3, 14))
        bh = int(rng.integers(3, 14))
        x = int(rng.integers(0, size - bw))
        y = int(rng.integers(0, size - bh))
        grid[y:y + bh, x:x + bw] = True
    # 1~3 个圆形障碍
    Y, X = np.ogrid[:size, :size]
    for _ in range(int(rng.integers(1, 4))):
        cx, cy = rng.integers(8, size - 8, 2)
        r = float(rng.uniform(3.0, 9.0))
        grid |= (X - cx) ** 2 + (Y - cy) ** 2 <= r * r
    return grid


def make_dataset(n_maps, size=64, per_map=None, seed=0):
    """随机地图 + 随机起终点，得到引导场回归数据集"""
    rng = np.random.default_rng(seed)
    feats_all, target_all = [], []
    rng_cells = 900  # 每张图随机采样 900 个栅格（含障碍/走廊/远区）

    for _ in range(n_maps):
        occ = random_map(rng, size)
        free = np.argwhere(~occ)
        if len(free) < 2:
            continue
        s, g = free[rng.integers(0, len(free), 2)]
        sy, sx = int(s[0]), int(s[1])
        gy, gx = int(g[0]), int(g[1])
        if math.hypot(sx - gx, sy - gy) < size * 0.4:
            continue

        feats = guidance_features(size, size, sx, sy, gx, gy, occ)
        target = analytic_guidance(size, size, sx, sy, gx, gy, occ).ravel()

        idx = rng.integers(0, size * size, rng_cells)
        # 强制包含起终点与走廊附近的点，避免数据集被远区 0 主导
        ys, xs = np.ogrid[:size, :size]
        corridor_mask = analytic_guidance(size, size, sx, sy, gx, gy, occ).ravel() > 0.05
        corridor_idx = np.flatnonzero(corridor_mask)
        if len(corridor_idx):
            extra = corridor_idx[rng.integers(0, len(corridor_idx),
                                              min(300, len(corridor_idx)))]
            idx = np.concatenate([idx, extra, np.array([sy * size + sx, gy * size + gx])])
        feats_all.append(feats[idx])
        target_all.append(target[idx].reshape(-1, 1))

    X = np.vstack(feats_all).astype(np.float64)
    Y = np.vstack(target_all).astype(np.float64)
    return X, Y


def main():
    parser = argparse.ArgumentParser(description="训练 Neural A* 引导场网络")
    parser.add_argument('--maps', type=int, default=160)
    parser.add_argument('--epochs', type=int, default=24)
    parser.add_argument('--batch-size', type=int, default=512)
    parser.add_argument('--lr', type=float, default=1e-3)
    parser.add_argument('--seed', type=int, default=2026)
    parser.add_argument('--out', type=str,
                        default=os.path.join(ROOT, 'rov_mujoco', 'planner_weights.npz'))
    args = parser.parse_args()

    print("=" * 66)
    print("  任务 3: Neural A* 引导场网络训练 (NumPy, 专家蒸馏)")
    print("=" * 66)
    print("  网络结构 : 13 -> 64 -> 32 -> 1 (tanh/sigmoid)")

    X, Y = make_dataset(args.maps, seed=args.seed)
    print(f"  样本数量 : {X.shape[0]} (来自 {args.maps} 张随机地图)")
    print(f"  目标均值 : {Y.mean():.6f} | 目标 >0.1 占比: {100.0 * np.mean(Y > 0.1):.2f}%")

    rng = np.random.default_rng(args.seed)
    idx = rng.permutation(len(X))
    n_val = max(1, int(len(X) * 0.2))
    val_idx, tr_idx = idx[:n_val], idx[n_val:]
    Xtr, Ytr = X[tr_idx], Y[tr_idx]
    Xva, Yva = X[val_idx], Y[val_idx]

    model = NumpyMLP([FEATURE_DIM, 64, 32, 1], hidden_act='tanh',
                     output_act='sigmoid', seed=args.seed)
    opt = Adam(lr=args.lr)
    history = {'train_loss': [], 'val_loss': []}

    for epoch in range(args.epochs):
        perm = rng.permutation(len(Xtr))
        losses = []
        for start in range(0, len(Xtr), args.batch_size):
            b = perm[start:start + args.batch_size]
            pred = model.forward(Xtr[b])
            losses.append(model.mse(pred, Ytr[b]))
            grads = model.backward(Ytr[b])
            opt.step(model.W + model.b,
                     [g[0] for g in grads] + [g[1] for g in grads])
        train_loss = float(np.mean(losses))
        val_loss = model.mse(model.forward(Xva), Yva)
        history['train_loss'].append(train_loss)
        history['val_loss'].append(val_loss)
        if epoch % 4 == 0 or epoch == args.epochs - 1:
            print(f"  epoch {epoch + 1:3d}/{args.epochs} | train MSE={train_loss:.6f} | "
                  f"val MSE={val_loss:.6f}")

    # 相关性与物理意义检查：引导场应在走廊上接近 1，远区接近 0
    pred = model.forward(Xva)[:, 0]
    target = Yva[:, 0]
    corr = float(np.corrcoef(pred, target)[0, 1])
    corridor = target > 0.2
    print("\n  ── 验证集结果 ──")
    print(f"  归一化 MSE : {model.mse(pred.reshape(-1, 1), target.reshape(-1, 1)):.6f}")
    print(f"  相关系数   : {corr:.5f}")
    print(f"  走廊区均值 : 预测={pred[corridor].mean():.4f} 专家={target[corridor].mean():.4f}")
    print(f"  远区均值   : 预测={pred[~corridor].mean():.6f} 专家={target[~corridor].mean():.6f}")
    print(f"  参数量     : {model.n_params()}")
    assert corr > 0.98, "引导场网络与专家先验相关性不足"
    assert pred[corridor].mean() > 0.5, "网络未能学到走廊高势能"

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    save_npz(args.out, model, extra={'val_mse': np.float32(history['val_loss'][-1])})
    print(f"\n  ✅ 权值已保存: {args.out}")
    print("=" * 66)


if __name__ == '__main__':
    main()
