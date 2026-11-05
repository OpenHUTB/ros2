#!/usr/bin/env python3
"""
离线训练 360 维 Autoencoder 用于激光雷达噪声修复。

合成数据 → 加噪 → 训练 Autoencoder → 保存权重。
无需 ROS 环境，纯 PyTorch + NumPy 即可运行。

用法：
    python3 train_autoencoder.py                  # 默认参数训练（纯合成数据，150 epoch）
    python3 train_autoencoder.py --epochs 150     # 显式指定训练轮数（与默认一致）
    python3 train_autoencoder.py --real-scans ../models/real_scans_sample.npz
                                                  # 混入 Gazebo 采集的真实扫描（推荐）
"""
import argparse
import os
import time
from typing import Tuple

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset


# ---------------------------------------------------------------------------
# Autoencoder 架构（与 main.py 中保持一致）
# ---------------------------------------------------------------------------
class LidarAutoencoder(nn.Module):
    """360 维激光雷达的 1D 卷积自编码器。

    用一维卷积而不是全连接，是因为雷达去噪本质上是**局部平滑**：
    相邻几束在同一个物体上、距离接近，噪声表现为逐束抖动。
    卷积的平移等变性天然适配这种结构，能在压掉逐束噪声的同时
    保留物体边界处的距离跳变；全连接层对每个角度单独学习，
    既学不到"邻域平滑"，参数量也大得多。

    结构（360 束 → 瓶颈特征图 → 360 束）：
        Conv1d(1→16,  k=5, s=2)     180
        Conv1d(16→32, k=5, s=2)      90
        Conv1d(32→64, k=5, s=2)      45   ← 瓶颈：45 × 64 特征图
        ConvTranspose1d(64→32)       90
        ConvTranspose1d(32→16)      180
        ConvTranspose1d(16→1)       360

    瓶颈宽度 64 是实测选出来的：更窄（32）时远距离扇区会被整体压向均值，
    下游左右比较会翻转；更宽则收益递减。
    """

    def __init__(self, input_dim: int = 360, latent_dim: int = 64):
        super().__init__()
        self.input_dim = input_dim
        self.latent_dim = latent_dim

        self.encoder = nn.Sequential(
            nn.Conv1d(1, 16, kernel_size=5, stride=2, padding=2),
            nn.ReLU(inplace=True),
            nn.Conv1d(16, 32, kernel_size=5, stride=2, padding=2),
            nn.ReLU(inplace=True),
            nn.Conv1d(32, latent_dim, kernel_size=5, stride=2, padding=2),
            nn.ReLU(inplace=True),
        )
        self.decoder = nn.Sequential(
            nn.ConvTranspose1d(latent_dim, 32, kernel_size=4, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.ConvTranspose1d(32, 16, kernel_size=4, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.ConvTranspose1d(16, 1, kernel_size=4, stride=2, padding=1),
            # 输出层不加激活，距离可为任意正实数；main.py 端统一 clip 到 [0.05, 3.5]
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x: (B, 360) → (B, 1, 360) 后走卷积，最后压回 (B, 360)
        y = self.decoder(self.encoder(x.unsqueeze(1)))
        return y.squeeze(1)


# ---------------------------------------------------------------------------
# 合成数据生成
# ---------------------------------------------------------------------------
def generate_open_space() -> np.ndarray:
    """开阔场景：大部分方向距离 2.0~3.5 m"""
    return np.random.uniform(2.0, 3.5, 360).astype(np.float32)


def generate_front_obstacle() -> np.ndarray:
    """正前方有障碍（0-20° + 340-360° 区域近）"""
    r = np.random.uniform(2.2, 3.5, 360).astype(np.float32)
    idx = np.concatenate([np.arange(0, 20), np.arange(340, 360)])
    r[idx] = np.random.uniform(0.25, 0.9, len(idx))
    return r


def generate_left_obstacle() -> np.ndarray:
    """左侧有障碍（20-80°）"""
    r = np.random.uniform(2.5, 3.5, 360).astype(np.float32)
    r[20:80] = np.random.uniform(0.2, 0.8, 60)
    return r


def generate_right_obstacle() -> np.ndarray:
    """右侧有障碍（280-340°）"""
    r = np.random.uniform(2.5, 3.5, 360).astype(np.float32)
    r[280:340] = np.random.uniform(0.2, 0.8, 60)
    return r


def generate_corridor() -> np.ndarray:
    """走廊：两侧近（90-270°），前后稍远"""
    r = np.random.uniform(2.0, 3.5, 360).astype(np.float32)
    r[90:270] = np.random.uniform(0.25, 0.55, 180)
    r[0:10] = np.random.uniform(1.5, 2.5, 10)
    r[350:360] = np.random.uniform(1.5, 2.5, 10)
    return r


def generate_wall_front() -> np.ndarray:
    """正前方一堵墙（0-10° 特别近）"""
    r = np.random.uniform(2.5, 3.5, 360).astype(np.float32)
    r[0:10] = np.random.uniform(0.15, 0.5, 10)
    return r


def generate_mixed_obstacles() -> np.ndarray:
    """混合障碍：随机 1~4 段近障碍区（不跨越 360° 边界）"""
    r = np.random.uniform(1.8, 3.5, 360).astype(np.float32)
    n_regions = np.random.randint(1, 5)
    for _ in range(n_regions):
        start = np.random.randint(0, 340)
        length = np.random.randint(5, min(30, 360 - start))
        # 取 min 避免 start+length > 360
        actual_len = min(length, 360 - start)
        r[start : start + actual_len] = np.random.uniform(0.15, 0.8, actual_len)
    return r


def generate_ring() -> np.ndarray:
    """模拟圆形房间/环岛：距离在 1.2~2.8 m"""
    return np.random.uniform(1.2, 2.8, 360).astype(np.float32)


CLEAN_SAMPLERS = [
    generate_open_space,
    generate_front_obstacle,
    generate_left_obstacle,
    generate_right_obstacle,
    generate_corridor,
    generate_wall_front,
    generate_mixed_obstacles,
    generate_ring,
]


def add_gaussian_noise(ranges: np.ndarray, sigma: float) -> np.ndarray:
    """加高斯噪声"""
    noisy = ranges + np.random.normal(0.0, sigma, ranges.shape).astype(np.float32)
    noisy = np.clip(noisy, 0.05, 3.5).astype(np.float32)
    return noisy


def add_salt_pepper(ranges: np.ndarray, amount: float = 0.1) -> np.ndarray:
    """椒盐噪声：随机把部分点设成 max 或 min"""
    noisy = ranges.copy()
    mask = np.random.choice([True, False], size=ranges.shape, p=[amount / 2, 1 - amount / 2])
    noisy[mask & (np.random.rand(*ranges.shape) > 0.5)] = 3.5
    noisy[mask & (np.random.rand(*ranges.shape) <= 0.5)] = 0.05
    return noisy


def add_sensor_drops(ranges: np.ndarray, drop_rate: float = 0.05) -> np.ndarray:
    """传感器丢点：部分点变成 NaN 等价的最大值"""
    noisy = ranges.copy()
    mask = np.random.random(ranges.shape) < drop_rate
    noisy[mask] = 3.5
    return noisy


NOISE_TYPES = ["gauss_low", "gauss_mid", "gauss_high",
               "saltpepper", "sensor_drop", "mixed"]


def apply_random_noise(clean: np.ndarray) -> np.ndarray:
    """对一帧干净扫描随机施加一种噪声，模拟传感器损坏。"""
    noise_type = np.random.choice(NOISE_TYPES)
    if noise_type == "gauss_low":
        return add_gaussian_noise(clean, sigma=0.15)
    if noise_type == "gauss_mid":
        return add_gaussian_noise(clean, sigma=0.3)
    if noise_type == "gauss_high":
        return add_gaussian_noise(clean, sigma=0.5)
    if noise_type == "saltpepper":
        return add_salt_pepper(clean, amount=0.12)
    if noise_type == "sensor_drop":
        return add_sensor_drops(clean, drop_rate=0.08)
    # mixed: 先高斯再丢点
    n = add_gaussian_noise(clean, sigma=0.25)
    return add_sensor_drops(n, drop_rate=0.05)


def synthesize_dataset(n_samples: int = 16000,
                       real_scans: np.ndarray = None,
                       real_ratio: float = 0.5) -> Tuple[np.ndarray, np.ndarray]:
    """合成 (corrupted, clean) 训练数据对。

    clean 样本有两个来源：
      1. 8 种几何场景的合成器（覆盖尖锐的扇区对比，保证离线自测可判）；
      2. 可选的真实仿真扫描（real_scans），保证训练分布包含 Gazebo
         里那种"近障碍密集 + 平滑距离梯度"的形态。

    真实扫描没有现成标签，但它本身就是干净数据，直接当 clean 样本用，
    再加合成噪声即可得到训练对。返回 shape 都是 (n_samples, 360)。
    """
    np.random.seed(42)
    torch.manual_seed(42)  # 固定网络初始化与批次顺序，保证每次训练结果可复现

    n_real = 0
    if real_scans is not None and len(real_scans) > 0 and real_ratio > 0:
        n_real = int(n_samples * real_ratio)
    n_synth = n_samples - n_real

    clean = []
    corrupted = []

    for _ in range(n_synth):
        # 近障碍场景加权采样（避障决策最依赖近距重构精度）
        sampler = CLEAN_SAMPLERS[np.random.choice(
            len(CLEAN_SAMPLERS),
            p=[0.15, 0.20, 0.15, 0.15, 0.10, 0.10, 0.05, 0.10])]
        c = sampler()
        clean.append(c)
        corrupted.append(apply_random_noise(c))

    for _ in range(n_real):
        # 有放回采样真实扫描：数据量有限但场景形态最贴近部署环境
        idx = np.random.randint(0, len(real_scans))
        c = real_scans[idx].astype(np.float32)
        clean.append(c)
        corrupted.append(apply_random_noise(c))

    order = np.random.permutation(len(clean))
    clean = np.array(clean, dtype=np.float32)[order]
    corrupted = np.array(corrupted, dtype=np.float32)[order]
    print("[train] 数据来源: 合成 %d + 真实 %d" % (n_synth, n_real))
    return corrupted, clean


# ---------------------------------------------------------------------------
# 损失函数
# ---------------------------------------------------------------------------
def weighted_mse(pred: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
    """近障碍区域加权。

    纯 MSE 会把近距障碍重构值平滑到中距离，导致避障阈值（0.8 m）漏检；
    分级加权：<0.5 m 的束 8 倍权重、0.5~1 m 的束 4 倍权重，
    显著改善近距障碍的重构保真度。
    """
    w = 1.0 + 3.0 * (target < 1.0).float() + 4.0 * (target < 0.5).float()
    return (w * (pred - target) ** 2).mean()


# ---------------------------------------------------------------------------
# 训练入口
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="训练激光雷达 Autoencoder")
    parser.add_argument("--epochs", type=int, default=150, help="训练轮数 (默认 150)")
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--samples", type=int, default=16000, help="合成样本数 (默认 16000)")
    parser.add_argument("--real-scans", type=str, default="",
                        help="真实仿真扫描 npz（键名 scans），可选；提供后按 --real-ratio 混入训练")
    parser.add_argument("--real-ratio", type=float, default=0.5,
                        help="真实扫描占训练集的比例 (默认 0.5)")
    parser.add_argument("--save-dir", type=str,
                        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models"),
                        help="权重保存目录 (默认 scripts/../models/)")
    args = parser.parse_args()

    save_dir = os.path.abspath(args.save_dir)
    os.makedirs(save_dir, exist_ok=True)
    save_path = os.path.join(save_dir, "autoencoder_weights.pt")

    device = torch.device("cpu")  # main.py 里固定用 CPU，这里保持一致
    print(f"[train] device = {device}, save_path = {save_path}")

    real_scans = None
    if args.real_scans:
        real_scans = np.load(args.real_scans)["scans"]
        print(f"[train] 载入真实扫描 {len(real_scans)} 帧 <- {args.real_scans}")

    # 1. 合成数据
    print(f"[train] 生成 {args.samples} 组 (corrupted, clean) 数据对 ...")
    corrupted_np, clean_np = synthesize_dataset(
        n_samples=args.samples, real_scans=real_scans, real_ratio=args.real_ratio)
    x = torch.tensor(corrupted_np)
    y = torch.tensor(clean_np)
    dataset = TensorDataset(x, y)
    loader = DataLoader(dataset, batch_size=args.batch_size, shuffle=True)
    print(f"[train] 数据集 ready: {len(dataset)} samples, {len(loader)} batches")

    # 2. 初始化模型
    model = LidarAutoencoder().to(device)
    optimizer = optim.Adam(model.parameters(), lr=args.lr)

    # 3. 训练循环
    model.train()
    t0 = time.time()
    for epoch in range(1, args.epochs + 1):
        running = 0.0
        for batch_x, batch_y in loader:
            batch_x = batch_x.to(device)
            batch_y = batch_y.to(device)

            optimizer.zero_grad()
            pred = model(batch_x)
            loss = weighted_mse(pred, batch_y)
            loss.backward()
            optimizer.step()

            running += loss.item() * batch_x.size(0)

        avg_loss = running / len(dataset)
        if epoch % 20 == 0 or epoch == 1:
            elapsed = time.time() - t0
            print(f"  epoch {epoch:3d}/{args.epochs}  MSE loss = {avg_loss:.6f}  "
                  f"({elapsed:.1f} s)")

    # 4. 保存权重
    model.eval()
    torch.save(model.state_dict(), save_path)
    elapsed = time.time() - t0
    print(f"\n[train] 训练完成，总耗时 {elapsed:.1f} s")
    print(f"[train] 权重已保存 -> {save_path}")

    # 5. 快速验证
    with torch.no_grad():
        sample_in = torch.tensor(corrupted_np[:1])
        sample_out = model(sample_in).numpy()[0]
        sample_ref = clean_np[0]
        err = np.mean((sample_out - sample_ref) ** 2)
        print(f"[train] 前 1 样本验证 MSE = {err:.6f} (越小越好)")


if __name__ == "__main__":
    main()
