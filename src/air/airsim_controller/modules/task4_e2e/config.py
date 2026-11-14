#!/usr/bin/env python3
"""task4_e2e 配置文件
端到端：深度相机图像 -> CNN -> (v, omega) 速度指令
"""
import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


class Config:
    """任务4参数"""

    # ---- 路径（类内，供 cfg.DATASET_DIR / cfg.WEIGHT_PATH 访问）----
    DATASET_DIR = os.path.join(BASE_DIR, "data")
    WEIGHT_PATH = os.path.join(DATASET_DIR, "e2e_cnn.pt")

    # ---- AirSim 通信 ----
    AIRSIM_IP = "127.0.0.1"          # 虚拟机内运行用 127.0.0.1；跨机改为宿主机 IP
    AIRSIM_PORT = 41451
    VEHICLE_NAME = ""                # 空串使用默认无人机

    # ---- 深度相机感知 ----
    CAMERA_NAME = "0"                # 前视相机
    MAX_RANGE = 10.0                 # 深度截断量程(m)，用于归一化
    IMG_H = 64                       # 下采样图像高度
    IMG_W = 64                       # 下采样图像宽度

    # ---- 控制 ----
    V_MAX = 1.0                      # 最大线速度 m/s
    W_MAX = 1.5                      # 最大角速度 rad/s
    CTRL_RATE = 10.0                 # 控制频率 Hz
    HOVER_RANGE = 0.30               # 深度归一化均值低于该值(前方 <3m)则减速悬停

    # ---- 数据采集 ----
    COLLECT_SECONDS = 120            # 采集时长(秒)，测试可改为 30

    # ---- CNN 训练 ----
    TRAIN_EPOCHS = 30
    TRAIN_LR = 1e-3
    TRAIN_BATCH = 32
    TRAIN_SAMPLES = 0                # 0=使用全部采集数据
    SEED = 42
