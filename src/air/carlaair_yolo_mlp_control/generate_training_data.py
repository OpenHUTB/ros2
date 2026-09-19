# -*- coding: utf-8 -*-

import csv
import random
import numpy as np


NUM_SAMPLES = 50000

KP_XY = 0.8
KP_Z = 0.8

KD_XY = 0.35
KD_Z = 0.35

MAX_XY_SPEED = 4.0
MAX_Z_SPEED = 2.0


def clip(value, limit):
    return max(-limit, min(limit, value))


with open(
    "controller_dataset.csv",
    "w",
    newline="",
    encoding="utf-8"
) as f:

    writer = csv.writer(f)

    writer.writerow([
        "ex",
        "ey",
        "ez",
        "vx",
        "vy",
        "vz",
        "vx_cmd",
        "vy_cmd",
        "vz_cmd"
    ])

    for _ in range(NUM_SAMPLES):

        # 模拟各种目标误差
        ex = random.uniform(-15, 15)
        ey = random.uniform(-15, 15)
        ez = random.uniform(-6, 6)

        # 模拟当前速度
        vx = random.uniform(-4, 4)
        vy = random.uniform(-4, 4)
        vz = random.uniform(-2, 2)

        # PD 专家控制器
        vx_cmd = (
            KP_XY * ex
            - KD_XY * vx
        )

        vy_cmd = (
            KP_XY * ey
            - KD_XY * vy
        )

        vz_cmd = (
            KP_Z * ez
            - KD_Z * vz
        )

        vx_cmd = clip(
            vx_cmd,
            MAX_XY_SPEED
        )

        vy_cmd = clip(
            vy_cmd,
            MAX_XY_SPEED
        )

        vz_cmd = clip(
            vz_cmd,
            MAX_Z_SPEED
        )

        writer.writerow([
            ex,
            ey,
            ez,
            vx,
            vy,
            vz,
            vx_cmd,
            vy_cmd,
            vz_cmd
        ])


print(
    "训练数据生成完成:",
    NUM_SAMPLES,
    "条"
)

print(
    "文件: controller_dataset.csv"
)