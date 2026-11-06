#!/usr/bin/env python3
"""离线训练脚本：行为克隆（监督回归）。

数据来自 collect 节点写出的 <data_dir>/controls.csv 与 frames/*.png，
按 8:1:1 划分训练/验证/测试集，训练完成后输出三个文件：

    model.pth       最佳（验证损失最低）权重
    loss_curve.png  训练/验证 MSE 曲线
    metrics.json    测试集 MSE / MAE、分维 MAE、样本量、耗时

用法：
    ros2 run uav_vision_nn train -- --epochs 50 --data-dir ~/.ros/uav_vision_nn/data
"""

import argparse
import copy
import csv
import json
import os
import random
import time

import cv2
import numpy as np
import torch
from torch.utils.data import DataLoader, Dataset, random_split

from uav_vision_nn.model import DrivingCNN, preprocess_image

DEFAULT_DATA_DIR = '~/.ros/uav_vision_nn/data'
DEFAULT_OUTPUT_DIR = '~/.ros/uav_vision_nn'


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='uav_vision_nn 行为克隆训练')
    parser.add_argument('--data-dir', default=DEFAULT_DATA_DIR, help='采集数据目录')
    parser.add_argument('--output-dir', default=DEFAULT_OUTPUT_DIR, help='权重与日志输出目录')
    parser.add_argument('--epochs', type=int, default=50, help='训练轮数')
    parser.add_argument('--batch-size', type=int, default=32, help='批大小')
    parser.add_argument('--lr', type=float, default=1e-3, help='Adam 学习率')
    parser.add_argument('--image-size', type=int, default=64, help='网络输入边长')
    parser.add_argument('--seed', type=int, default=42, help='随机种子，保证可复现')
    parser.add_argument('--device', default='cpu', help='训练设备，cpu 或 cuda')
    return parser.parse_args(argv)


class FrameDataset(Dataset):
    """把 CSV 里的每一行读成 (1, S, S) 图像张量与 (2,) 标签张量。

    图像在构造时一次性载入内存：64x64 单通道只有 4 KB 一张，几千帧完全放得下，
    这样可以避免每个 epoch 反复读盘把训练时间浪费在 IO 上。
    """

    def __init__(self, data_dir, image_size, log=print):
        self.samples = []
        csv_path = os.path.join(data_dir, 'controls.csv')
        if not os.path.isfile(csv_path):
            raise FileNotFoundError(f'找不到标签文件: {csv_path}')

        skipped = 0
        with open(csv_path, newline='', encoding='utf-8') as handle:
            for row in csv.DictReader(handle):
                rel_path = (row.get('image') or '').strip()
                if not rel_path:
                    continue
                image_path = os.path.join(data_dir, rel_path)
                image = cv2.imread(image_path, cv2.IMREAD_COLOR)
                if image is None:
                    skipped += 1
                    continue
                image = preprocess_image(image, image_size)[0]
                label = np.array([float(row['vx']), float(row['vy'])], dtype=np.float32)
                self.samples.append((torch.from_numpy(image), torch.from_numpy(label)))

        log(f'读取 {csv_path}: 有效样本 {len(self.samples)} 帧，跳过损坏 {skipped} 帧')
        if not self.samples:
            raise ValueError('没有可用样本，请先运行 collect 节点采集数据')

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, index):
        return self.samples[index]


def split_dataset(dataset, seed):
    """8:1:1 划分，比例由样本量取整决定，剩余样本全部给测试集。"""
    total = len(dataset)
    if total < 10:
        raise ValueError(f'样本量 {total} 太少，至少需要 10 帧')
    train_len = int(total * 0.8)
    rest = total - train_len
    val_len = rest // 2
    test_len = rest - val_len
    if val_len == 0 or test_len == 0:
        raise ValueError(f'样本量 {total} 不足以划分验证/测试集')

    generator = torch.Generator().manual_seed(seed)
    train_set, val_set, test_set = random_split(
        dataset, [train_len, val_len, test_len], generator=generator)
    return train_set, val_set, test_set


def evaluate(model, loader, loss_fn, device):
    """返回 MSE、平均绝对误差与分维 MAE（单位 m/s）。"""
    model.eval()
    total_loss = 0.0
    total_abs = 0.0
    abs_vx = 0.0
    abs_vy = 0.0
    count = 0
    with torch.no_grad():
        for images, labels in loader:
            images = images.to(device)
            labels = labels.to(device)
            predictions = model(images)
            batch = images.size(0)
            total_loss += loss_fn(predictions, labels).item() * batch
            error = (predictions - labels).abs()
            total_abs += error.sum().item()
            abs_vx += error[:, 0].sum().item()
            abs_vy += error[:, 1].sum().item()
            count += batch
    return {
        'mse': total_loss / count,
        'mae': total_abs / (2.0 * count),
        'mae_vx': abs_vx / count,
        'mae_vy': abs_vy / count,
    }


def plot_curve(epochs, train_losses, val_losses, path):
    """画训练曲线，无显示环境下列表后端，避免依赖 X11。"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(figsize=(6, 4))
    axes.plot(epochs, train_losses, label='train MSE')
    axes.plot(epochs, val_losses, label='val MSE')
    axes.set_xlabel('epoch')
    axes.set_ylabel('MSE (m/s)^2')
    axes.set_yscale('log')
    axes.set_title('uav_vision_nn behavior cloning')
    axes.grid(True, which='both', alpha=0.3)
    axes.legend()
    figure.tight_layout()
    figure.savefig(path, dpi=120)
    plt.close(figure)


def main(argv=None):
    args = parse_args(argv)
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    data_dir = os.path.expanduser(args.data_dir)
    output_dir = os.path.expanduser(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    dataset = FrameDataset(data_dir, args.image_size)
    train_set, val_set, test_set = split_dataset(dataset, args.seed)

    shuffle_generator = torch.Generator().manual_seed(args.seed + 1)
    train_loader = DataLoader(train_set, batch_size=args.batch_size,
                              shuffle=True, generator=shuffle_generator)
    val_loader = DataLoader(val_set, batch_size=args.batch_size, shuffle=False)
    test_loader = DataLoader(test_set, batch_size=args.batch_size, shuffle=False)

    device = torch.device(args.device)
    model = DrivingCNN(image_size=args.image_size).to(device)
    loss_fn = torch.nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)

    print(f'训练集 {len(train_set)} / 验证集 {len(val_set)} / 测试集 {len(test_set)}')
    print(f'网络参数量 {model.num_parameters()}，设备 {device}，epochs {args.epochs}')

    best_state = copy.deepcopy(model.state_dict())
    best_val_loss = float('inf')
    best_epoch = 0
    train_losses = []
    val_losses = []
    begin = time.time()

    for epoch in range(1, args.epochs + 1):
        model.train()
        running_loss = 0.0
        seen = 0
        for images, labels in train_loader:
            images = images.to(device)
            labels = labels.to(device)
            predictions = model(images)
            loss = loss_fn(predictions, labels)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            running_loss += loss.item() * images.size(0)
            seen += images.size(0)

        train_loss = running_loss / max(seen, 1)
        val_metrics = evaluate(model, val_loader, loss_fn, device)
        train_losses.append(train_loss)
        val_losses.append(val_metrics['mse'])

        if val_metrics['mse'] < best_val_loss:
            best_val_loss = val_metrics['mse']
            best_epoch = epoch
            best_state = copy.deepcopy(model.state_dict())

        if epoch == 1 or epoch % 5 == 0 or epoch == args.epochs:
            print(f'epoch {epoch:3d}/{args.epochs} '
                  f'train_mse={train_loss:.5f} val_mse={val_metrics["mse"]:.5f} '
                  f'val_mae={val_metrics["mae"]:.4f}')

    train_seconds = time.time() - begin

    model.load_state_dict(best_state)
    test_metrics = evaluate(model, test_loader, loss_fn, device)

    model_path = os.path.join(output_dir, 'model.pth')
    model.save(model_path)
    curve_path = os.path.join(output_dir, 'loss_curve.png')
    plot_curve(list(range(1, len(train_losses) + 1)), train_losses, val_losses, curve_path)

    metrics = {
        'package': 'uav_vision_nn',
        'task': 'behavior_cloning',
        'seed': args.seed,
        'device': str(device),
        'epochs': args.epochs,
        'batch_size': args.batch_size,
        'learning_rate': args.lr,
        'optimizer': 'Adam',
        'loss': 'MSELoss',
        'image_size': args.image_size,
        'parameters': model.num_parameters(),
        'samples': {
            'total': len(dataset),
            'train': len(train_set),
            'val': len(val_set),
            'test': len(test_set),
        },
        'best_epoch': best_epoch,
        'train_seconds': round(train_seconds, 2),
        'final_train_mse': train_losses[-1],
        'best_val_mse': best_val_loss,
        'test': test_metrics,
        'files': {'model': model_path, 'loss_curve': curve_path},
    }
    metrics_path = os.path.join(output_dir, 'metrics.json')
    with open(metrics_path, 'w', encoding='utf-8') as handle:
        json.dump(metrics, handle, ensure_ascii=False, indent=2)

    print('训练完成：')
    print(f'  最优 epoch {best_epoch}，验证 MSE {best_val_loss:.5f}')
    print(f'  测试 MSE {test_metrics["mse"]:.5f} m^2/s^2，'
          f'MAE {test_metrics["mae"]:.4f} m/s'
          f'（vx {test_metrics["mae_vx"]:.4f}，vy {test_metrics["mae_vy"]:.4f}）')
    print(f'  权重 {model_path}')
    print(f'  曲线 {curve_path}')
    print(f'  指标 {metrics_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
