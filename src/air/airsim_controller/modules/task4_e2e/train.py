#!/usr/bin/env python3
"""train.py —— 端到端 CNN 行为克隆训练（深度图 -> 速度指令）

结构：
    Conv2d(1,16,3)->ReLU->MaxPool(2)
    ->Conv2d(16,32,3)->ReLU->MaxPool(2)
    ->Flatten->Linear->ReLU->Linear(2)->Tanh
输入：64x64 归一化深度图 (N,1,64,64)
输出：(v_norm, w_norm) ∈ [-1,1]^2

运行： python3 train.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from config import Config

try:
    import torch
    import torch.nn as nn
    _TORCH_OK = True
except ImportError:  # pragma: no cover
    torch = None
    nn = None
    _TORCH_OK = False


class E2ECNN(nn.Module):
    """轻量 CNN：深度图 -> 动作。"""

    def __init__(self, in_h=64, in_w=64):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(1, 16, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),
            nn.Conv2d(16, 32, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),
        )
        # 64 -> 32 -> 16
        flat = 32 * (in_h // 4) * (in_w // 4)
        self.head = nn.Sequential(
            nn.Linear(flat, 128), nn.ReLU(),
            nn.Linear(128, 2), nn.Tanh(),
        )

    def forward(self, x):
        return self.head(self.features(x).flatten(1))


def main():
    if not _TORCH_OK:
        print("[错误] 训练需要 PyTorch：pip install torch")
        sys.exit(1)

    cfg = Config()
    data_path = os.path.join(cfg.DATASET_DIR, "e2e_data.npz")
    if not os.path.exists(data_path):
        print(f"[错误] 未找到数据集 {data_path}，请先运行 python3 collect_data.py")
        sys.exit(1)

    data = np.load(data_path)
    X, Y = data["images"], data["actions"]
    print(f"加载数据集: {X.shape[0]} 帧, 图像 {X.shape[1:]}")

    if cfg.TRAIN_SAMPLES and cfg.TRAIN_SAMPLES < len(X):
        idx = np.random.default_rng(cfg.SEED).choice(len(X), cfg.TRAIN_SAMPLES, replace=False)
        X, Y = X[idx], Y[idx]

    # (N,64,64) -> (N,1,64,64)
    X = X.reshape(-1, 1, cfg.IMG_H, cfg.IMG_W)
    Xt = torch.from_numpy(X.astype(np.float32))
    Yt = torch.from_numpy(Y.astype(np.float32))

    model = E2ECNN(cfg.IMG_H, cfg.IMG_W)
    opt = torch.optim.Adam(model.parameters(), lr=cfg.TRAIN_LR)
    loss_fn = nn.MSELoss()
    from torch.utils.data import DataLoader, TensorDataset
    loader = DataLoader(TensorDataset(Xt, Yt), batch_size=cfg.TRAIN_BATCH, shuffle=True)

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
        if (epoch + 1) % 5 == 0:
            print(f"  epoch {epoch+1:3d}/{cfg.TRAIN_EPOCHS}  loss={total/len(X):.5f}")

    os.makedirs(cfg.DATASET_DIR, exist_ok=True)
    torch.save(model.state_dict(), cfg.WEIGHT_PATH)
    print(f"[完成] 权重已保存: {cfg.WEIGHT_PATH}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
