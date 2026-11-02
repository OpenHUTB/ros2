#!/usr/bin/env python3
"""train.py —— 行为克隆训练神经网络探索策略

流程：
    1) 用专家规则生成训练数据（样本：扇区特征+目标方向 -> 速度指令）
    2) 训练 MLP（ExplorerNet）
    3) 保存权重到 data/explorer_net.pt

专家规则（用于生成标签）：
    - 最近障碍物距离 < 安全距离：减速并转向远离障碍
    - 否则：按目标方向偏航误差比例转向，全速前进

运行： python3 train.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from config import Config
from explorer_net import ExplorerNet, _TORCH_OK


def expert_policy(features, goal_dir, cfg):
    """专家策略：生成监督标签 (v_norm, w_norm)。
    features/goal_dir 均为归一化值；避障阈值 = SAFE_DIST / SECTOR_MAX
    """
    d_min = features.min()
    sector = int(features.argmin())
    angle_obs = -np.pi + (sector + 0.5) * (2 * np.pi / cfg.NUM_SECTORS)

    # 目标方向偏航误差（机体前方为0，右偏为正）
    # goal_dir = [前向分量, 右向分量] = [cosθ, sinθ]
    theta_goal = np.arctan2(goal_dir[1], goal_dir[0])
    # w>0 为左转：目标在右侧(θ>0)应右转(w<0)，故取负号
    w_raw = np.clip(-cfg.YAW_RATE_K * theta_goal, -cfg.W_MAX, cfg.W_MAX)

    avoid_thr = cfg.SAFE_DIST / cfg.SECTOR_MAX
    if d_min < avoid_thr:
        # 避障：减速 + 转向远离障碍
        v_norm = -0.8
        w_norm = -np.sign(angle_obs) * 1.0 if abs(angle_obs) > 1e-3 else 1.0
    else:
        v_norm = 1.0
        w_norm = w_raw / cfg.W_MAX
    return v_norm, w_norm


def generate_dataset(cfg, n_samples):
    """生成与运行时分布匹配的训练样本。

    运行时扇区特征绝大多数为满量程（归一化 1.0），
    仅当无人机靠近障碍时部分扇区变小。
    """
    rng = np.random.default_rng(cfg.SEED)
    X, Y = [], []
    for _ in range(n_samples):
        # 绝大多数扇区无数据（1.0），50% 概率出现一个障碍扇区
        features = np.full(cfg.NUM_SECTORS, 1.0, dtype=np.float64)
        if rng.random() < 0.5:
            k = int(rng.integers(0, cfg.NUM_SECTORS))
            features[k] = rng.uniform(0.05, 0.5)   # 障碍距离 0.4~4.0m
        goal_dir = rng.uniform(-1, 1, size=2)
        norm = np.linalg.norm(goal_dir) + 1e-6
        goal_dir = goal_dir / norm

        v_norm, w_norm = expert_policy(features, goal_dir, cfg)
        X.append(np.concatenate([features, goal_dir]))
        Y.append([v_norm, w_norm])
    return np.array(X, dtype=np.float32), np.array(Y, dtype=np.float32)


def main():
    if not _TORCH_OK:
        print("[错误] 训练需要 PyTorch：pip install torch")
        sys.exit(1)

    import torch
    import torch.nn as nn
    from torch.utils.data import DataLoader, TensorDataset

    cfg = Config()
    print(f"生成专家数据集 {cfg.TRAIN_SAMPLES} 条 ...")
    X, Y = generate_dataset(cfg, cfg.TRAIN_SAMPLES)

    net = ExplorerNet(cfg.INPUT_DIM, cfg.HIDDEN_DIM, cfg.OUTPUT_DIM)
    model = net.model
    opt = torch.optim.Adam(model.parameters(), lr=cfg.TRAIN_LR)
    loss_fn = nn.MSELoss()

    loader = DataLoader(TensorDataset(torch.from_numpy(X), torch.from_numpy(Y)),
                        batch_size=cfg.TRAIN_BATCH, shuffle=True)

    print("开始训练 ...")
    for epoch in range(cfg.TRAIN_EPOCHS):
        total = 0.0
        for xb, yb in loader:
            opt.zero_grad()
            out = model(xb)
            loss = loss_fn(out, yb)
            loss.backward()
            opt.step()
            total += loss.item() * len(xb)
        if (epoch + 1) % 10 == 0:
            print(f"  epoch {epoch+1:3d}/{cfg.TRAIN_EPOCHS}  loss={total/len(X):.5f}")

    os.makedirs(os.path.dirname(cfg.WEIGHT_PATH), exist_ok=True)
    net.save(cfg.WEIGHT_PATH)
    print(f"[完成] 权重已保存: {cfg.WEIGHT_PATH}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
