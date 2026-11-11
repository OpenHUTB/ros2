#!/usr/bin/env python3
"""训练脚本：从采集到的深度图 + 参考偏差回归专家速度。

    python3 -m uav_depth_track_nn.train \
        --data-dir ~/.ros/uav_depth_track_nn/data \
        --output-dir ~/.ros/uav_depth_track_nn/model \
        --epochs 60 --batch-size 16

产物：
    <output_dir>/model.pth        权重（``main`` 默认加载这个）
    <output_dir>/metrics.json     样本数、参数量、训练/验证/测试误差、耗时
    <output_dir>/loss_curve.png   训练/验证 MSE 曲线，可直接贴进文档
"""

import argparse
import csv
import io
import json
import os
import random
import time

import cv2
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset, random_split

from uav_depth_track_nn.model import (DEFAULT_IMAGE_SIZE, DEFAULT_MAX_DEPTH,
                                      TrackNet, preprocess_depth)

FEATURE_COLUMNS = ['dx', 'dy', 'sin_err', 'cos_err']


class TrackDataset(Dataset):
    """(深度图, 4 维参考特征) -> (vx, vy)。"""

    def __init__(self, rows, data_dir, image_size, max_depth):
        self._rows = rows
        self._data_dir = data_dir
        self._image_size = image_size
        self._max_depth = max_depth

    def __len__(self):
        return len(self._rows)

    def __getitem__(self, index):
        row = self._rows[index]
        path = os.path.join(self._data_dir, row['image'])
        # 16 位毫米 PNG，原样读入后换算成米
        raw = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        if raw is None:
            raise FileNotFoundError(f'读不到深度图: {path}')
        depth_m = raw.astype(np.float32) / 1000.0

        image = torch.from_numpy(
            preprocess_depth(depth_m, self._image_size, self._max_depth))
        features = torch.tensor(
            [float(row[c]) for c in FEATURE_COLUMNS], dtype=torch.float32)
        label = torch.tensor(
            [float(row['vx']), float(row['vy'])], dtype=torch.float32)
        return image[0], features, label


def load_rows(csv_path):
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(
            f'找不到标签文件 {csv_path}，请先运行 collect.launch.py 采集数据')
    with io.open(csv_path, encoding='utf-8') as handle:
        rows = [row for row in csv.DictReader(handle) if row.get('image')]
    if not rows:
        raise ValueError(f'{csv_path} 里没有样本')
    return rows


def evaluate(model, loader):
    model.eval()
    total, count = 0.0, 0
    abs_sum = 0.0
    with torch.no_grad():
        for image, features, label in loader:
            pred = model(image, features)
            total += float(nn.functional.mse_loss(pred, label, reduction='sum'))
            abs_sum += float((pred - label).abs().sum())
            count += label.numel() // 2
    # reduction='sum' 得到的是"所有样本 × 两个分量"的平方和，
    # count 是样本数，因此每个分量的 MSE 要除以 2*count；MAE 同理。
    mse = total / (2.0 * count)
    return mse, abs_sum / (2.0 * count)


def save_loss_curve(history, path):
    """不依赖 matplotlib：直接画一张 640x360 的 PNG，省掉训练环境的额外依赖。"""
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return None

    width, height, margin = 640, 360, 56
    image = Image.new('RGB', (width, height), 'white')
    draw = ImageDraw.Draw(image)
    draw.rectangle([margin, margin, width - margin, height - margin],
                   outline=(60, 60, 60))

    epochs = [point['epoch'] for point in history]
    train = [point['train_mse'] for point in history]
    valid = [point['val_mse'] for point in history]
    highest = max(train + valid) or 1.0
    lowest = min(train + valid)

    def to_xy(epoch, value):
        fx = (epoch - epochs[0]) / max(epochs[-1] - epochs[0], 1)
        fy = (value - lowest) / max(highest - lowest, 1e-6)
        return (margin + fx * (width - 2 * margin),
                height - margin - fy * (height - 2 * margin))

    for series, color in ((train, (30, 90, 200)), (valid, (220, 90, 40))):
        draw.line([to_xy(e, v) for e, v in zip(epochs, series)], fill=color, width=2)

    draw.text((margin, 18), 'train (blue) / val (orange) MSE', fill=(20, 20, 20))
    draw.text((width - 150, height - 30), 'epoch', fill=(20, 20, 20))
    image.save(path)
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description='训练深度相机跟踪网络')
    parser.add_argument('--data-dir', default='~/.ros/uav_depth_track_nn/data')
    parser.add_argument('--output-dir', default='~/.ros/uav_depth_track_nn/model')
    parser.add_argument('--epochs', type=int, default=60)
    parser.add_argument('--batch-size', type=int, default=16)
    parser.add_argument('--lr', type=float, default=1e-3)
    parser.add_argument('--val-ratio', type=float, default=0.1)
    parser.add_argument('--test-ratio', type=float, default=0.1)
    parser.add_argument('--image-size', type=int, default=DEFAULT_IMAGE_SIZE)
    parser.add_argument('--max-depth', type=float, default=DEFAULT_MAX_DEPTH)
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args(argv)

    data_dir = os.path.expanduser(args.data_dir)
    output_dir = os.path.expanduser(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    rows = load_rows(os.path.join(data_dir, 'controls.csv'))
    # 固定全部随机源：划分、打乱顺序、权重初始化都要可复现，
    # 否则同一个 --seed 跑两次会得到不同的测试指标。
    torch.manual_seed(args.seed)
    random.seed(args.seed)
    np.random.seed(args.seed)

    dataset = TrackDataset(rows, data_dir, args.image_size, args.max_depth)

    total = len(dataset)
    test_size = max(int(round(total * args.test_ratio)), 1) if total >= 3 else 0
    val_size = max(int(round(total * args.val_ratio)), 1) if total - test_size >= 2 else 0
    train_size = total - test_size - val_size
    generator = torch.Generator().manual_seed(args.seed)
    train_set, val_set, test_set = random_split(
        dataset, [train_size, val_size, test_size], generator=generator)

    train_loader = DataLoader(
        train_set, batch_size=args.batch_size, shuffle=True,
        generator=torch.Generator().manual_seed(args.seed))
    val_loader = DataLoader(val_set, batch_size=args.batch_size)
    test_loader = DataLoader(test_set, batch_size=args.batch_size)

    model = TrackNet(image_size=args.image_size)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    criterion = nn.MSELoss()

    best_val = float('inf')
    best_path = os.path.join(output_dir, 'model.pth')
    history = []
    started = time.time()

    for epoch in range(1, args.epochs + 1):
        model.train()
        running = 0.0
        for image, features, label in train_loader:
            optimizer.zero_grad()
            loss = criterion(model(image, features), label)
            loss.backward()
            optimizer.step()
            running += float(loss) * image.shape[0]
        train_mse = running / max(len(train_set), 1)

        val_mse = 0.0
        if val_size:
            val_mse, _ = evaluate(model, val_loader)
            if val_mse < best_val:
                best_val = val_mse
                model.save(best_path)

        history.append({'epoch': epoch, 'train_mse': train_mse, 'val_mse': val_mse})
        print(f'epoch {epoch:3d}/{args.epochs} train_mse={train_mse:.4f} '
              f'val_mse={val_mse:.4f}')

    if val_size == 0:
        model.save(best_path)

    test_mse, test_mae = (evaluate(model, test_loader) if test_size
                          else (float('nan'), float('nan')))
    elapsed = time.time() - started

    curve_path = save_loss_curve(history, os.path.join(output_dir, 'loss_curve.png'))
    metrics = {
        'samples': total,
        'split': {'train': train_size, 'val': val_size, 'test': test_size},
        'parameters': model.num_parameters(),
        'epochs': args.epochs,
        'seed': args.seed,
        'image_size': args.image_size,
        'max_depth': args.max_depth,
        'val_ratio': args.val_ratio,
        'test_ratio': args.test_ratio,
        'data_dir': data_dir,
        'batch_size': args.batch_size,
        'learning_rate': args.lr,
        'best_val_mse': best_val,
        'final_train_mse': history[-1]['train_mse'] if history else None,
        'test_mse': test_mse,
        'test_mae': test_mae,
        'elapsed_seconds': round(elapsed, 2),
        'loss_curve': curve_path,
    }
    with io.open(os.path.join(output_dir, 'metrics.json'), 'w', encoding='utf-8') as handle:
        json.dump(metrics, handle, ensure_ascii=False, indent=2)

    print(f'训练完成：{total} 个样本，参数 {metrics["parameters"]}，耗时 {elapsed:.1f} s')
    print(f'测试 MSE {test_mse:.4f} (m/s)^2，MAE {test_mae:.4f} m/s')
    print(f'权重 {best_path}，指标 {os.path.join(output_dir, "metrics.json")}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
