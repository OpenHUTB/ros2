#!/usr/bin/env python3
"""训练 MLP 神经网络：输入前视灰度图 -> 输出速度控制 (vx, vy)"""
import os
import csv
import numpy as np
import cv2
import torch
import torch.nn as nn
import torch.optim as optim

DATA_DIR = os.path.expanduser("~/ros2/src/uav_vision_nn/data")
MODEL_PATH = os.path.expanduser("~/ros2/src/uav_vision_nn/model.pth")

# 1. 加载数据
images, labels = [], []
with open(os.path.join(DATA_DIR, "controls.csv")) as f:
    reader = csv.DictReader(f)
    for row in reader:
        img = cv2.imread(os.path.join(DATA_DIR, row["image"]), cv2.IMREAD_GRAYSCALE)
        img = cv2.resize(img, (64, 64)).astype(np.float32) / 255.0
        images.append(img.flatten())
        labels.append([float(row["vx"]), float(row["vy"])])

X = torch.tensor(np.array(images))
Y = torch.tensor(np.array(labels))
print(f"数据加载完成：{len(X)} 帧，输入维度 {X.shape[1]}")

# 2. MLP 模型
class MLP(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(64*64, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 2)
        )
    def forward(self, x):
        return self.net(x)

model = MLP()
optimizer = optim.Adam(model.parameters(), lr=1e-3)
loss_fn = nn.MSELoss()

# 3. 训练
for epoch in range(100):
    pred = model(X)
    loss = loss_fn(pred, Y)
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()
    if epoch % 20 == 0:
        print(f"Epoch {epoch:3d}  Loss: {loss.item():.6f}")

torch.save(model.state_dict(), MODEL_PATH)
print(f"模型已保存到 {MODEL_PATH}")
