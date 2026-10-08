#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
train.py —— 训练 CNN 感知模型（行为克隆）
"""
import os
import argparse
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, random_split

from dataset import ImageDataset
from model import build_model


def train(csv_path, epochs=30, batch_size=16, lr=1e-3, out_dir="models"):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"训练设备: {device}")

    dataset = ImageDataset(csv_path)
    if len(dataset) < 10:
        raise RuntimeError(f"样本太少（仅 {len(dataset)} 条），先多采集一些图像数据。")

    n_val = max(1, int(0.15 * len(dataset)))
    n_train = len(dataset) - n_val
    train_set, val_set = random_split(dataset, [n_train, n_val])
    train_loader = DataLoader(train_set, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_set, batch_size=batch_size, shuffle=False)

    model = build_model().to(device)
    optim = torch.optim.Adam(model.parameters(), lr=lr)
    criterion = nn.MSELoss()

    os.makedirs(out_dir, exist_ok=True)
    best_val = float("inf")

    for epoch in range(1, epochs + 1):
        model.train()
        train_loss = 0.0
        for imgs, actions in train_loader:
            imgs = imgs.to(device)
            actions = actions.to(device)
            optim.zero_grad()
            pred = model(imgs)
            loss = criterion(pred, actions)
            loss.backward()
            optim.step()
            train_loss += loss.item() * imgs.size(0)
        train_loss /= len(train_set)

        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for imgs, actions in val_loader:
                imgs = imgs.to(device)
                actions = actions.to(device)
                pred = model(imgs)
                val_loss += criterion(pred, actions).item() * imgs.size(0)
        val_loss /= len(val_set)

        print(f"Epoch {epoch:02d}/{epochs}  train_loss={train_loss:.5f}  val_loss={val_loss:.5f}")

        if val_loss < best_val:
            best_val = val_loss
            torch.save({"state_dict": model.state_dict()},
                       os.path.join(out_dir, "best_drone_model.pth"))
            print(f"  -> 已保存最佳权重 (val_loss={best_val:.5f})")

    print(f"训练完成。最佳 val_loss = {best_val:.5f}，权重在 {out_dir}/best_drone_model.pth")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=os.path.expanduser("~/perception_dataset/data.csv"))
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--batch-size", type=int, default=16)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--out-dir", default=os.path.join(os.path.dirname(__file__), "models"))
    args = ap.parse_args()
    train(args.csv, args.epochs, args.batch_size, args.lr, args.out_dir)
