#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 训练脚本：train（任务四：端到端视觉行为克隆，第二阶段·训练）
# 作用：用 record_dataset 采集的数据训练 DronePolicyNet，MSE 回归 4 维动作
#       [vx, vy, vz, yaw_rate]。要点：
#   1. 自动检测 CUDA / CPU（torch.cuda.is_available()）；
#   2. MSE 损失 + Adam 优化器，默认 30 轮（--epochs 可调，建议 20~30）；
#   3. 每个 Epoch 打印 train_loss / val_loss 与用时；
#   4. 只保存验证集 Loss 最小的权重到 models/best_drone_model.pth
#      （保存为 checkpoint 字典，含 state_dict/epoch/val_loss/img_size，
#      部署节点两种格式都兼容）；
#   5. 8:2 划分训练/验证集（固定种子 42，可复现）。
# 用法（仓库根目录运行，与 record_dataset 默认输出路径对齐）：
#   python src/air/air_learning/train.py
#   python src/air/air_learning/train.py --data data/dataset --epochs 20

import argparse
import math
import os
import time

import torch
import torch.nn as nn

from dataset import IMG_SIZE, make_dataloaders
from model import build_model


def train_one_epoch(model, loader, criterion, optimizer, device):
    """训练一轮：返回该轮平均损失（按样本数加权平均）"""
    model.train()
    total_loss = 0.0
    for images, labels in loader:
        images, labels = images.to(device), labels.to(device)
        optimizer.zero_grad()
        loss = criterion(model(images), labels)
        loss.backward()
        optimizer.step()
        total_loss += loss.item() * images.size(0)
    return total_loss / len(loader.dataset)


@torch.no_grad()
def evaluate(model, loader, criterion, device):
    """验证一轮：返回平均损失（no_grad + eval，不更新权重）"""
    model.eval()
    total_loss = 0.0
    for images, labels in loader:
        images, labels = images.to(device), labels.to(device)
        total_loss += criterion(model(images), labels).item() * images.size(0)
    return total_loss / len(loader.dataset)


def main():
    parser = argparse.ArgumentParser(
        description='任务四行为克隆训练：图像 -> [vx,vy,vz,yaw_rate] 回归')
    parser.add_argument('--data', default='data/dataset',
                        help='数据集目录（含 labels.csv 与 images/），默认 data/dataset')
    parser.add_argument('--epochs', type=int, default=30,
                        help='训练轮数，默认 30（建议 20~30）')
    parser.add_argument('--batch-size', type=int, default=16,
                        help='批大小，默认 16')
    parser.add_argument('--lr', type=float, default=1e-3,
                        help='Adam 学习率，默认 1e-3')
    parser.add_argument('--out', default='models/best_drone_model.pth',
                        help='最优权重保存路径，默认 models/best_drone_model.pth')
    parser.add_argument('--num-workers', type=int, default=0,
                        help='DataLoader 子进程数，默认 0（Windows 兼容）')
    parser.add_argument('--seed', type=int, default=42,
                        help='划分随机种子，默认 42（保证 8:2 划分可复现）')
    parser.add_argument('--val-ratio', type=float, default=0.2,
                        help='验证集占比，默认 0.2')
    args = parser.parse_args()

    # 设备：CUDA 可用则用 GPU，否则 CPU
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    if device.type == 'cuda':
        torch.backends.cudnn.benchmark = True        # 固定输入尺寸下加速卷积

    train_loader, val_loader = make_dataloaders(
        args.data, val_ratio=args.val_ratio, seed=args.seed,
        batch_size=args.batch_size, num_workers=args.num_workers)

    model = build_model().to(device)
    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)

    print('=' * 60)
    print('  任务四：端到端视觉行为克隆 —— 模型训练')
    print('=' * 60)
    print('  设备      : %s' % device)
    print('  数据集    : %s' % os.path.abspath(args.data))
    print('  训练/验证 : %d / %d 帧（%.0f:%.0f 划分，种子 %d）'
          % (len(train_loader.dataset), len(val_loader.dataset),
             (1 - args.val_ratio) * 100, args.val_ratio * 100, args.seed))
    print('  输入尺寸  : %dx%d | 输出 4 维 [vx,vy,vz,yaw_rate]'
          % (IMG_SIZE, IMG_SIZE))
    print('  超参      : epochs=%d, batch=%d, lr=%g, Adam+MSE'
          % (args.epochs, args.batch_size, args.lr))
    print('  权重输出  : %s' % os.path.abspath(args.out))
    print('-' * 60)

    best_val = math.inf
    best_epoch = 0
    for epoch in range(1, args.epochs + 1):
        t0 = time.time()
        train_loss = train_one_epoch(model, train_loader, criterion,
                                     optimizer, device)
        val_loss = evaluate(model, val_loader, criterion, device)
        elapsed = time.time() - t0

        improved = val_loss < best_val
        if improved:                      # 验证集 Loss 创新低才落盘覆盖
            best_val = val_loss
            best_epoch = epoch
            out_dir = os.path.dirname(args.out)
            if out_dir:
                os.makedirs(out_dir, exist_ok=True)
            torch.save({
                'state_dict': model.state_dict(),
                'epoch': epoch,
                'val_loss': val_loss,
                'img_size': IMG_SIZE,
                'num_outputs': 4,
            }, args.out)

        print('Epoch %d/%d | train_loss: %.4f | val_loss: %.4f | 用时 %.1f s%s'
              % (epoch, args.epochs, train_loss, val_loss, elapsed,
                 '  <- 最优，已保存' if improved else ''))

    print('-' * 60)
    print('训练完成：最优验证 Loss = %.4f（Epoch %d），权重已保存到 %s'
          % (best_val, best_epoch, os.path.abspath(args.out)))
    print('部署：启动 drone_autonomous_node.py 后按回车进入自主巡航')
    print('=' * 60)


if __name__ == '__main__':
    main()
