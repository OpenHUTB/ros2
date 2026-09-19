# -*- coding: utf-8 -*-

import csv
import numpy as np

import torch
import torch.nn as nn
from torch.utils.data import (
    TensorDataset,
    DataLoader,
    random_split
)


# ============================================================
# 配置
# ============================================================

DATASET_PATH = "controller_dataset.csv"

MODEL_PATH = "mlp_controller.pth"

EPOCHS = 80
BATCH_SIZE = 256
LEARNING_RATE = 0.001


# 输入归一化尺度
INPUT_SCALE = np.array([
    15.0,   # ex
    15.0,   # ey
    6.0,    # ez
    4.0,    # vx
    4.0,    # vy
    2.0     # vz
], dtype=np.float32)


# 输出归一化尺度
OUTPUT_SCALE = np.array([
    4.0,    # vx_cmd
    4.0,    # vy_cmd
    2.0     # vz_cmd
], dtype=np.float32)


# ============================================================
# 神经网络
# ============================================================

class MLPController(nn.Module):

    def __init__(self):

        super().__init__()

        self.net = nn.Sequential(

            nn.Linear(6, 64),
            nn.ReLU(),

            nn.Linear(64, 64),
            nn.ReLU(),

            nn.Linear(64, 32),
            nn.ReLU(),

            nn.Linear(32, 3),

            nn.Tanh()
        )

    def forward(self, x):
        return self.net(x)


# ============================================================
# 读取 CSV
# ============================================================

data = np.loadtxt(
    DATASET_PATH,
    delimiter=",",
    skiprows=1,
    dtype=np.float32
)


X = data[:, 0:6]

Y = data[:, 6:9]


# 归一化
X = X / INPUT_SCALE
Y = Y / OUTPUT_SCALE


X = torch.tensor(
    X,
    dtype=torch.float32
)

Y = torch.tensor(
    Y,
    dtype=torch.float32
)


dataset = TensorDataset(
    X,
    Y
)


train_size = int(
    len(dataset) * 0.8
)

val_size = (
    len(dataset)
    - train_size
)


train_dataset, val_dataset = random_split(
    dataset,
    [
        train_size,
        val_size
    ]
)


train_loader = DataLoader(
    train_dataset,
    batch_size=BATCH_SIZE,
    shuffle=True
)

val_loader = DataLoader(
    val_dataset,
    batch_size=BATCH_SIZE
)


# ============================================================
# GPU
# ============================================================

device = torch.device(
    "cuda"
    if torch.cuda.is_available()
    else "cpu"
)


print("训练设备:", device)

if torch.cuda.is_available():
    print(
        "GPU:",
        torch.cuda.get_device_name(0)
    )


# ============================================================
# 模型
# ============================================================

model = MLPController().to(
    device
)


criterion = nn.MSELoss()


optimizer = torch.optim.Adam(
    model.parameters(),
    lr=LEARNING_RATE
)


# ============================================================
# 训练
# ============================================================

for epoch in range(
    1,
    EPOCHS + 1
):

    model.train()

    train_loss = 0.0


    for x_batch, y_batch in train_loader:

        x_batch = x_batch.to(device)
        y_batch = y_batch.to(device)

        optimizer.zero_grad()

        output = model(
            x_batch
        )

        loss = criterion(
            output,
            y_batch
        )

        loss.backward()

        optimizer.step()

        train_loss += (
            loss.item()
            * x_batch.size(0)
        )


    train_loss /= len(
        train_dataset
    )


    # 验证
    model.eval()

    val_loss = 0.0


    with torch.no_grad():

        for x_batch, y_batch in val_loader:

            x_batch = x_batch.to(
                device
            )

            y_batch = y_batch.to(
                device
            )

            output = model(
                x_batch
            )

            loss = criterion(
                output,
                y_batch
            )

            val_loss += (
                loss.item()
                * x_batch.size(0)
            )


    val_loss /= len(
        val_dataset
    )


    if (
        epoch == 1
        or epoch % 5 == 0
    ):

        print(
            f"Epoch {epoch:3d}/{EPOCHS}",
            f"Train={train_loss:.6f}",
            f"Val={val_loss:.6f}"
        )


# ============================================================
# 保存
# ============================================================

torch.save(
    {
        "model_state_dict":
            model.state_dict(),

        "input_scale":
            INPUT_SCALE,

        "output_scale":
            OUTPUT_SCALE
    },

    MODEL_PATH
)


print()
print(
    "训练完成"
)

print(
    "模型:",
    MODEL_PATH
)