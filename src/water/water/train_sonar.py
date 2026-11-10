"""声呐图像 CNN 训练脚本（使用真实数据集，按类别子文件夹组织）。

数据集目录示例：
    dataset/
    ├── rock/*.jpg
    ├── pipe/*.jpg
    └── sand/*.jpg
子文件夹名按字母序即为类别索引，训练结束会把该顺序写入 weights/classes.json，
推理节点默认读取它，无需手工对齐。

用法：
    ros2 run water train_sonar --data ./dataset --epochs 20 --out src/water/weights/sonar_cnn_weights.pth
"""

import argparse
import json
import os
import random

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, Subset
from torchvision import datasets, transforms

from water.sonar_model import SonarCNN


def parse_args():
    parser = argparse.ArgumentParser(description="训练声呐图像分类 CNN")
    parser.add_argument("--data", required=True, help="数据集根目录（ImageFolder 结构）")
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--input-size", type=int, default=256)
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=42)
    # 默认输出到功能包内 weights/，colcon build 时按 glob 装进 share/water/weights
    parser.add_argument("--out", default=os.path.join("weights", "sonar_cnn_weights.pth"))
    parser.add_argument("--device", default="auto", choices=["auto", "cpu", "cuda"])
    return parser.parse_args()


def build_loaders(args):
    train_tf = transforms.Compose([
        transforms.Resize((args.input_size, args.input_size)),
        transforms.RandomHorizontalFlip(),
        transforms.ToTensor(),               # 转成 0~1 的 RGB 张量，与推理端 /255 一致
    ])
    val_tf = transforms.Compose([
        transforms.Resize((args.input_size, args.input_size)),
        transforms.ToTensor(),
    ])

    base = datasets.ImageFolder(args.data)   # 仅用于读取类别与样本总数
    if len(base.classes) < 2:
        raise RuntimeError("数据集类别数不足 2，请检查子文件夹结构")
    idx = list(range(len(base)))
    random.Random(args.seed).shuffle(idx)

    n_val = int(len(idx) * args.val_ratio)
    val_idx, train_idx = idx[:n_val], idx[n_val:]

    train_set = datasets.ImageFolder(args.data, transform=train_tf)
    val_set = datasets.ImageFolder(args.data, transform=val_tf)
    train_set = Subset(train_set, train_idx)
    val_set = Subset(val_set, val_idx)

    # 划分后训练集必须仍包含全部类别，否则 CrossEntropy 无法正常收敛
    train_labels = {base.targets[i] for i in train_idx}
    if len(train_labels) < 2:
        raise RuntimeError(
            "训练集类别数不足 2，请检查子文件夹结构或减小 --val-ratio")

    train_loader = DataLoader(train_set, batch_size=args.batch_size, shuffle=True, num_workers=2)
    val_loader = DataLoader(val_set, batch_size=args.batch_size, shuffle=False, num_workers=2)
    return base.classes, train_loader, val_loader


def evaluate(model, loader, criterion, device):
    model.eval()
    total, correct, loss_sum = 0, 0, 0.0
    with torch.no_grad():
        for img, label in loader:
            img, label = img.to(device), label.to(device)
            out = model(img)
            loss_sum += criterion(out, label).item() * img.size(0)
            correct += (out.argmax(dim=1) == label).sum().item()
            total += img.size(0)
    return loss_sum / max(total, 1), correct / max(total, 1)


def main():
    args = parse_args()
    random.seed(args.seed)
    torch.manual_seed(args.seed)

    if args.device == "auto":
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    else:
        device = torch.device(args.device)

    classes, train_loader, val_loader = build_loaders(args)
    print(f"类别: {classes}")
    print(f"class_to_idx: { {c: i for i, c in enumerate(classes)} }")
    print(f"训练集 {len(train_loader.dataset)} 张, 验证集 {len(val_loader.dataset)} 张, device={device}")

    model = SonarCNN(num_classes=len(classes), input_size=args.input_size).to(device)
    criterion = nn.CrossEntropyLoss()
    opt = optim.Adam(model.parameters(), lr=args.lr)

    best_acc = 0.0
    out_path = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    for ep in range(args.epochs):
        model.train()
        loss_sum, total = 0.0, 0
        for img, label in train_loader:
            img, label = img.to(device), label.to(device)
            loss = criterion(model(img), label)
            opt.zero_grad()
            loss.backward()
            opt.step()
            loss_sum += loss.item() * img.size(0)
            total += img.size(0)

        val_loss, val_acc = evaluate(model, val_loader, criterion, device)
        print(f"Epoch {ep + 1}/{args.epochs} | train_loss {loss_sum / total:.4f} "
              f"| val_loss {val_loss:.4f} | val_acc {val_acc:.4f}")

        if val_acc >= best_acc:
            best_acc = val_acc
            torch.save(model.state_dict(), out_path)
            print(f"  已保存最优权重 -> {out_path} (val_acc={best_acc:.4f})")

    print(f"训练完成，最优验证准确率 {best_acc:.4f}，权重: {out_path}")

    # 类别顺序与权重一起保存：ImageFolder 按文件夹名排序，模型输出下标依赖这个顺序，
    # 推理节点默认读取该文件，避免用户自己填 classes 顺序填错导致类别标错
    classes_json = os.path.join(os.path.dirname(out_path), "classes.json")
    with open(classes_json, "w", encoding="utf-8") as f:
        json.dump(classes, f, ensure_ascii=False, indent=2)
    print(f"类别顺序已保存 -> {classes_json}")


if __name__ == "__main__":
    main()
