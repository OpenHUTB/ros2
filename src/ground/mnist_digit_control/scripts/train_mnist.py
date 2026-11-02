#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""训练 MNIST 手写数字 CNN。

用法（在模块目录下）：
    python3 scripts/train_mnist.py [--epochs 3] [--seed 42] [--data-dir data/mnist]

- 数据：自动从 ossci-datasets / Google 镜像下载 MNIST（约 11 MB），存入 data/mnist；
- 输出：models/mnist_cnn.pth（权重）与 artifacts/ 下的 loss 曲线、混淆矩阵、指标 json；
- 环境：CPU 即可，Ubuntu 20.04 + Python 3.8 + torch 2.4.1 实测 6 线程约 2 分钟。
"""
import argparse
import gzip
import json
import math
import os
import sys
import time
import urllib.request

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from mnist_digit_control.model import DigitCNN  # noqa: E402

import torch  # noqa: E402
import torch.nn as nn  # noqa: E402
import torch.nn.functional as F  # noqa: E402
from torch.utils.data import DataLoader, TensorDataset  # noqa: E402

MIRRORS = [
    "https://ossci-datasets.s3.amazonaws.com/mnist/",
    "https://storage.googleapis.com/cvdf-datasets/mnist/",
]
FILES = [
    "train-images-idx3-ubyte.gz",
    "train-labels-idx1-ubyte.gz",
    "t10k-images-idx3-ubyte.gz",
    "t10k-labels-idx1-ubyte.gz",
]

MODULE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def download_mnist(data_dir: str) -> None:
    os.makedirs(data_dir, exist_ok=True)
    for name in FILES:
        path = os.path.join(data_dir, name)
        if os.path.isfile(path) and os.path.getsize(path) > 1000:
            continue
        last_err = None
        for base in MIRRORS:
            try:
                print(f"下载 {base}{name}")
                urllib.request.urlretrieve(base + name, path)
                last_err = None
                break
            except Exception as exc:  # noqa: BLE001
                last_err = exc
        if last_err is not None:
            raise RuntimeError(f"MNIST 下载失败（可手动下载 4 个 .gz 放入 {data_dir}）: {last_err}")


def load_idx_images(path: str) -> np.ndarray:
    with gzip.open(path, "rb") as fh:
        data = np.frombuffer(fh.read(), np.uint8, offset=16)
    return data.reshape(-1, 28, 28).astype(np.float32) / 255.0


def load_idx_labels(path: str) -> np.ndarray:
    with gzip.open(path, "rb") as fh:
        return np.frombuffer(fh.read(), np.uint8, offset=8).astype(np.int64)


def evaluate(model, loader, device) -> tuple:
    model.eval()
    correct, total = 0, 0
    confusion = np.zeros((10, 10), dtype=np.int64)
    with torch.no_grad():
        for x, y in loader:
            pred = model(x.to(device)).argmax(dim=1).cpu().numpy()
            y = y.numpy()
            correct += int((pred == y).sum())
            total += len(y)
            for t, p in zip(y, pred):
                confusion[t, p] += 1
    return correct / total, confusion


def augment(xb: torch.Tensor) -> torch.Tensor:
    """模拟真实相机成像退化：随机旋转/缩放/平移 + 像素化重采样 + 模糊 + 噪声。

    目的是弥合“干净 MNIST”与“摄像头拍摄的手写数字”之间的域差距，
    使模型在视频/相机输入上仍能给出高置信度预测。
    """
    n = xb.shape[0]
    ang = (torch.rand(n) - 0.5) * math.radians(50)   # ±25°
    scale = 0.8 + 0.4 * torch.rand(n)
    tx = (torch.rand(n) - 0.5) * 0.2
    ty = (torch.rand(n) - 0.5) * 0.2
    cos, sin = torch.cos(ang) * scale, torch.sin(ang) * scale
    theta = torch.zeros(n, 2, 3)
    theta[:, 0, 0], theta[:, 0, 1], theta[:, 0, 2] = cos, -sin, tx
    theta[:, 1, 0], theta[:, 1, 1], theta[:, 1, 2] = sin, cos, ty
    grid = F.affine_grid(theta, xb.shape, align_corners=False)
    xb = F.grid_sample(xb, grid, align_corners=False, padding_mode="zeros")

    # 随机像素化（模拟先降采样再上采样的相机重采样）
    if torch.rand(1).item() < 0.5:
        factor = int(torch.randint(2, 5, (1,)).item())  # 2~4 倍降采样
        xb = F.interpolate(xb, scale_factor=1.0 / factor, mode="area")
        xb = F.interpolate(xb, size=(28, 28), mode="bilinear", align_corners=False)

    # 随机 3x3 高斯模糊（[1,2,1] 二项式近似，迭代 1~2 次）
    if torch.rand(1).item() < 0.6:
        row = torch.tensor([1.0, 2.0, 1.0])
        row = row / row.sum()
        kernel = (row[:, None] * row[None, :]).reshape(1, 1, 3, 3)
        for _ in range(int(torch.randint(1, 3, (1,)).item())):
            xb = F.conv2d(xb, kernel, padding=1)

    # 随机高斯噪声
    noise_amp = 0.05 * torch.rand(n, 1, 1, 1)
    xb = xb + torch.randn_like(xb) * noise_amp
    return xb.clamp(0.0, 1.0)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--epochs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--data-dir", default=os.path.join(MODULE_DIR, "data", "mnist"))
    args = parser.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    torch.set_num_threads(max(1, torch.get_num_threads()))
    device = "cpu"
    print(f"torch {torch.__version__}, 线程数 {torch.get_num_threads()}, 设备 {device}")

    download_mnist(args.data_dir)
    x_train = load_idx_images(os.path.join(args.data_dir, FILES[0]))
    y_train = load_idx_labels(os.path.join(args.data_dir, FILES[1]))
    x_test = load_idx_images(os.path.join(args.data_dir, FILES[2]))
    y_test = load_idx_labels(os.path.join(args.data_dir, FILES[3]))
    print(f"训练集 {x_train.shape[0]} 张，测试集 {x_test.shape[0]} 张")

    train_loader = DataLoader(
        TensorDataset(torch.from_numpy(x_train).unsqueeze(1), torch.from_numpy(y_train)),
        batch_size=args.batch_size, shuffle=True, num_workers=0,
    )
    test_loader = DataLoader(
        TensorDataset(torch.from_numpy(x_test).unsqueeze(1), torch.from_numpy(y_test)),
        batch_size=512, shuffle=False, num_workers=0,
    )

    model = DigitCNN().to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    criterion = nn.CrossEntropyLoss()

    losses, accs = [], []
    t_start = time.time()
    for epoch in range(1, args.epochs + 1):
        model.train()
        running, seen = 0.0, 0
        for x, y in train_loader:
            optimizer.zero_grad()
            loss = criterion(model(augment(x)), y)
            loss.backward()
            optimizer.step()
            running += float(loss) * len(y)
            seen += len(y)
        acc, _ = evaluate(model, test_loader, device)
        losses.append(running / seen)
        accs.append(acc)
        print(f"epoch {epoch}/{args.epochs}  loss={losses[-1]:.4f}  test_acc={acc:.4f}")
    train_seconds = time.time() - t_start

    test_acc, confusion = evaluate(model, test_loader, device)
    print(f"最终测试集准确率: {test_acc:.4f}（训练耗时 {train_seconds:.0f} s）")

    os.makedirs(os.path.join(MODULE_DIR, "models"), exist_ok=True)
    artifact_dir = os.path.join(MODULE_DIR, "artifacts")
    os.makedirs(artifact_dir, exist_ok=True)
    torch.save(model.state_dict(), os.path.join(MODULE_DIR, "models", "mnist_cnn.pth"))

    metrics = {
        "epochs": args.epochs, "seed": args.seed, "batch_size": args.batch_size,
        "lr": args.lr, "loss_per_epoch": losses, "test_acc_per_epoch": accs,
        "final_test_acc": test_acc, "train_seconds": round(train_seconds, 1),
        "params": int(sum(p.numel() for p in model.parameters())),
        "confusion": confusion.tolist(),
        "torch_version": torch.__version__,
    }
    with open(os.path.join(artifact_dir, "metrics.json"), "w", encoding="utf-8") as fh:
        json.dump(metrics, fh, ensure_ascii=False, indent=2)

    plot_curves(losses, accs, artifact_dir)
    plot_confusion(confusion, test_acc, artifact_dir)
    print(f"权重与图表已保存到 models/ 与 artifacts/")


def plot_curves(losses, accs, out_dir) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax1 = plt.subplots(figsize=(6, 3.6), dpi=120)
    epochs = range(1, len(losses) + 1)
    ax1.plot(epochs, losses, "o-", color="#d62728", label="train loss")
    ax1.set_xlabel("epoch")
    ax1.set_ylabel("loss", color="#d62728")
    ax2 = ax1.twinx()
    ax2.plot(epochs, accs, "s-", color="#1f77b4", label="test acc")
    ax2.set_ylabel("test accuracy", color="#1f77b4")
    ax2.set_ylim(0.9, 1.0)
    fig.suptitle("DigitCNN training on MNIST")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "loss_curve.png"))
    plt.close(fig)


def plot_confusion(confusion, test_acc, out_dir) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(5.2, 4.6), dpi=120)
    norm = confusion / np.maximum(confusion.sum(axis=1, keepdims=True), 1)
    im = ax.imshow(norm, cmap="Blues", vmin=0, vmax=1)
    ax.set_xticks(range(10), range(10))
    ax.set_yticks(range(10), range(10))
    ax.set_xlabel("predicted")
    ax.set_ylabel("true")
    for i in range(10):
        for j in range(10):
            if confusion[i, j]:
                ax.text(j, i, str(confusion[i, j]), ha="center", va="center",
                        fontsize=7, color="white" if norm[i, j] > 0.5 else "black")
    ax.set_title(f"MNIST test confusion matrix (acc={test_acc:.4f})")
    fig.colorbar(im, fraction=0.046)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "confusion_matrix.png"))
    plt.close(fig)


if __name__ == "__main__":
    main()
