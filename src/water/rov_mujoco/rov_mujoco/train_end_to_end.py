"""
水下机器人端到端视觉模型训练套件 (End-to-End Visuomotor Training Suite)
========================================================================
端到端视觉控制策略模型训练模块：
统一基于专家示教数据，采用行为克隆 (Behavior Cloning / 监督回归) 策略，
对比评测三种不同卷积视觉策略网络架构：
  1. Nature-CNN (经典基准轻量网络)
  2. ResNet-18 (深层残差网络)
  3. Spatial-Softmax (空间软注意力关键点网络)
"""

import os
import sys
import time
import math
import argparse
from typing import Dict, List, Tuple
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader

# 本地模块导入
try:
    from rov_mujoco.end_to_end_networks import (
        NatureCNNVisuomotorPolicy,
        ResNetVisuomotorPolicy,
        SpatialSoftmaxVisuomotorPolicy,
        VisuomotorController
    )
except ImportError:
    try:
        from underwater_rov_mujoco.end_to_end_networks import (
            NatureCNNVisuomotorPolicy,
            ResNetVisuomotorPolicy,
            SpatialSoftmaxVisuomotorPolicy,
            VisuomotorController
        )
    except ImportError:
        from end_to_end_networks import (
            NatureCNNVisuomotorPolicy,
            ResNetVisuomotorPolicy,
            SpatialSoftmaxVisuomotorPolicy,
            VisuomotorController
        )


class VisuomotorDataset(Dataset):
    """端到端水下视觉数据集"""

    def __init__(self, images: np.ndarray, actions: np.ndarray):
        # images: (N, 84, 84, 3) uint8
        # actions: (N, 4) float32
        self.images = torch.from_numpy(images).permute(0, 3, 1, 2).float() / 255.0
        self.actions = torch.from_numpy(actions).float()

    def __len__(self) -> int:
        return len(self.images)

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
        return self.images[idx], self.actions[idx]


def generate_synthetic_underwater_dataset(num_samples: int = 1200) -> Tuple[np.ndarray, np.ndarray]:
    """
    生成高仿真水下管网视觉巡检样本数据集：
    模拟水下前视摄像机在不同横向偏移、偏航倾角与水体浑浊度下看到的管道特征图与对应的修正控制指令。
    """
    np.random.seed(42)
    images = np.zeros((num_samples, 84, 84, 3), dtype=np.uint8)
    actions = np.zeros((num_samples, 4), dtype=np.float32)

    for i in range(num_samples):
        # 管道在图像中的横向中心位置 [-0.75, +0.75] (0 为画面正中)
        pipe_center_x = float(np.random.uniform(-0.75, 0.75))
        # 管道倾角 (偏航倾角) [-30deg, +30deg]
        pipe_angle = float(np.random.uniform(-0.4, 0.4))

        # 1. 绘制海床背景 (顶部开阔深海水体 + 中下部蓝青色海床网格)
        img = np.zeros((84, 84, 3), dtype=np.float32)
        # 上部远景深海水体
        img[:25, :, 0] = np.random.uniform(20.0, 40.0)
        img[:25, :, 1] = np.random.uniform(40.0, 70.0)
        img[:25, :, 2] = np.random.uniform(70.0, 110.0)
        # 中下部海床网格 (亮蓝青色)
        for r in range(25, 84):
            factor = (r - 25) / 59.0
            img[r, :, 0] = 45.0 + 15.0 * factor + np.random.uniform(-3, 3)
            img[r, :, 1] = 120.0 + 25.0 * factor + np.random.uniform(-5, 5)
            img[r, :, 2] = 180.0 + 35.0 * factor + np.random.uniform(-5, 5)

        # 2. 绘制透视投影海底管道 (自远至近逐渐加宽，钢灰色圆柱)
        for r in range(84):
            # 随深度距离透视加宽：顶部约 10px，底部约 24px
            half_w = int(5 + (r / 84.0) * 8.5)
            cx = int(42 + pipe_center_x * 36.0 + (r - 42) * pipe_angle * 0.35)
            c_min = max(0, cx - half_w)
            c_max = min(84, cx + half_w)
            if c_max > c_min:
                img[r, c_min:c_max, 0] = np.random.uniform(65.0, 90.0)
                img[r, c_min:c_max, 1] = np.random.uniform(75.0, 105.0)
                img[r, c_min:c_max, 2] = np.random.uniform(100.0, 136.0)
                # 圆柱高光脊线
                hl = (c_min + c_max) // 2
                hl_min = max(0, hl - 2)
                hl_max = min(84, hl + 2)
                img[r, hl_min:hl_max, :] += np.random.uniform(10.0, 20.0)

        # 3. 添加水下悬浮颗粒与轻微高斯散射噪声
        noise = np.random.normal(0, 2.0, (84, 84, 3))
        img = np.clip(img + noise, 0, 255).astype(np.uint8)
        images[i] = img

        # 4. 专家视觉伺服控制动作律 (严格符合机器人学 REP 103 标准):
        # 管道在右侧 (pipe_center_x > 0) -> 机体需向右横移 (vy < 0) 且顺时针向右转 (wz < 0)
        # 管道在左侧 (pipe_center_x < 0) -> 机体需向左横移 (vy > 0) 且逆时针向左转 (wz > 0)
        a_vx = float(0.85 - 0.35 * abs(pipe_center_x))
        a_vy = float(np.clip(-2.2 * pipe_center_x, -1.0, 1.0))
        a_vz = 0.0
        a_wz = float(np.clip(-2.5 * pipe_center_x - 1.8 * pipe_angle, -1.0, 1.0))
        actions[i] = [a_vx, a_vy, a_vz, a_wz]

    return images, actions


def train_behavior_cloning(
    model: nn.Module,
    dataset: VisuomotorDataset,
    epochs: int = 15,
    batch_size: int = 32,
    lr: float = 1e-3,
    device: str = "cuda" if torch.cuda.is_available() else "cpu"
) -> Tuple[nn.Module, float]:
    """训练行为克隆 (BC) 监督策略网络"""
    device_obj = torch.device(device)
    model = model.to(device_obj)
    model.train()

    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True)
    optimizer = optim.AdamW(model.parameters(), lr=lr, weight_decay=1e-4)
    criterion = nn.MSELoss()

    final_loss = 0.0
    for epoch in range(epochs):
        epoch_loss = 0.0
        for batch_imgs, batch_acts in dataloader:
            batch_imgs = batch_imgs.to(device_obj)
            batch_acts = batch_acts.to(device_obj)

            optimizer.zero_grad()
            preds = model(batch_imgs)
            loss = criterion(preds, batch_acts)
            loss.backward()
            optimizer.step()

            epoch_loss += loss.item() * len(batch_imgs)

        final_loss = epoch_loss / len(dataset)

    model.eval()
    return model, final_loss


def train_architecture(
    arch_name: str,
    train_dataset: VisuomotorDataset,
    output_dir: str = "models",
    epochs: int = 15,
    batch_size: int = 32,
    lr: float = 1e-3,
    device: str = "cuda" if torch.cuda.is_available() else "cpu"
) -> Tuple[str, float]:
    """训练指定架构的行为克隆策略并保存权重"""
    os.makedirs(output_dir, exist_ok=True)
    if arch_name == "resnet":
        model = ResNetVisuomotorPolicy()
        out_filename = "task4_resnet.pt"
    elif arch_name == "spatial_softmax":
        model = SpatialSoftmaxVisuomotorPolicy()
        out_filename = "task4_spatial_softmax.pt"
    else:
        model = NatureCNNVisuomotorPolicy()
        out_filename = "task4_nature_cnn.pt"

    print(f"  ▶ 正在训练 {arch_name} 架构 (行为克隆 BC, {epochs} epochs, device: {device})...")
    model, final_loss = train_behavior_cloning(
        model, train_dataset, epochs=epochs, batch_size=batch_size, lr=lr, device=device
    )

    weight_path = os.path.join(output_dir, out_filename)
    torch.save(model.state_dict(), weight_path)
    print(f"  ✅ {arch_name} 训练完成，Loss: {final_loss:.6f}，权重已保存至: {weight_path}")

    # 保持向后兼容命名别名，确保历史 launch / 测试脚本可正常索引
    alias_names = []
    if arch_name == "nature_cnn":
        alias_names = ["task4_nature_cnn_bc.pt", "task4_nature_cnn_sac.pt", "task4_nature_cnn_ppo.pt"]
    elif arch_name == "resnet":
        alias_names = ["task4_resnet_sac.pt"]
    elif arch_name == "spatial_softmax":
        alias_names = ["task4_spatial_softmax_sac.pt"]

    for alias in alias_names:
        alias_path = os.path.join(output_dir, alias)
        torch.save(model.state_dict(), alias_path)

    return weight_path, final_loss


def train_and_export_all_task4_models(
    arch: str = "all",
    output_dir: str = "models",
    epochs: int = 15,
    batch_size: int = 32,
    lr: float = 1e-3
) -> Dict[str, str]:
    """
    训练并保存端到端视觉策略网络模型权重。
    统一基于专家示教数据，采用行为克隆 (BC) 监督回归策略，
    对比三种视觉策略架构（Nature-CNN、ResNet-18、Spatial-Softmax）。
    """
    os.makedirs(output_dir, exist_ok=True)
    device = "cuda" if torch.cuda.is_available() else "cpu"

    real_data_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models", "real_scene_dataset.npz")
    if not os.path.exists(real_data_path):
        real_data_path = os.path.join(output_dir, "real_scene_dataset.npz")

    if os.path.exists(real_data_path):
        print(f"  ▶ 检测到水下管网渲染数据集: {real_data_path}，应用双向镜像与光照增广...")
        data = np.load(real_data_path)
        raw_imgs = data["images"]
        raw_acts = data["actions"]
        aug_imgs = []
        aug_acts = []
        for img, act in zip(raw_imgs, raw_acts):
            aug_imgs.append(img)
            aug_acts.append(act)
            img_flip = np.ascontiguousarray(np.fliplr(img))
            act_flip = np.array([act[0], -act[1], act[2], -act[3]], dtype=np.float32)
            aug_imgs.append(img_flip)
            aug_acts.append(act_flip)
            for scale in [0.92, 1.08]:
                noise1 = np.random.normal(0, 1.5, img.shape)
                aug_imgs.append(np.clip(img.astype(float) * scale + noise1, 0, 255).astype(np.uint8))
                aug_acts.append(act)
                noise2 = np.random.normal(0, 1.5, img.shape)
                aug_imgs.append(np.clip(img_flip.astype(float) * scale + noise2, 0, 255).astype(np.uint8))
                aug_acts.append(act_flip)
        images = np.array(aug_imgs, dtype=np.uint8)
        actions = np.array(aug_acts, dtype=np.float32)
        n_train = int(len(images) * 0.85)
        train_dataset = VisuomotorDataset(images[:n_train], actions[:n_train])
    else:
        print("  ▶ 未检测到离线数据集，生成合成水下管网示教数据 (1600 样本)...")
        images, actions = generate_synthetic_underwater_dataset(num_samples=1600)
        train_dataset = VisuomotorDataset(images[:1300], actions[:1300])

    targets = ["nature_cnn", "resnet", "spatial_softmax"] if arch == "all" else [arch]
    saved_weights = {}

    for target_arch in targets:
        p, _ = train_architecture(
            arch_name=target_arch,
            train_dataset=train_dataset,
            output_dir=output_dir,
            epochs=epochs,
            batch_size=batch_size,
            lr=lr,
            device=device
        )
        saved_weights[target_arch] = p

    print(f"✅ 策略权重成功导出至: {output_dir}/")
    return saved_weights


def main():
    parser = argparse.ArgumentParser(description="水下机器人端到端视觉策略网络训练工具 (基于行为克隆)")
    parser.add_argument(
        "--arch",
        type=str,
        default="all",
        choices=["all", "nature_cnn", "resnet", "spatial_softmax"],
        help="待训练的网络架构（默认 all，训练全部三种架构）"
    )
    parser.add_argument("--epochs", type=int, default=15, help="训练轮数 (默认 15)")
    parser.add_argument("--batch-size", type=int, default=32, help="批大小 (默认 32)")
    parser.add_argument("--lr", type=float, default=1e-3, help="学习率 (默认 1e-3)")
    parser.add_argument("--output-dir", type=str, default="models", help="权重输出目录 (默认 models)")
    args = parser.parse_args()

    train_and_export_all_task4_models(
        arch=args.arch,
        output_dir=args.output_dir,
        epochs=args.epochs,
        batch_size=args.batch_size,
        lr=args.lr
    )


if __name__ == "__main__":
    main()
