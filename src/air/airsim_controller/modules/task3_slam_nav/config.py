#!/usr/bin/env python3
"""task3_slam_nav 配置文件
所有可调参数集中于此，便于多场景测试时统一修改。
"""
import os

# ---- 路径 ----
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
WEIGHT_PATH = os.path.join(BASE_DIR, "data", "explorer_net.pt")
LOG_DIR = os.path.join(BASE_DIR, "data")


class Config:
    """任务3参数"""

    # ---- AirSim 通信 ----
    AIRSIM_IP = "127.0.0.1"          # 虚拟机内运行 AirSim 时使用 127.0.0.1；跨机改为宿主机 IP
    AIRSIM_PORT = 41451
    VEHICLE_NAME = ""                 # 空串使用默认无人机
    SIM_MODE = "Multirotor"

    # ---- LiDAR 感知 ----
    LIDAR_NAME = "Lidar1"
    NUM_SECTORS = 16                  # 360° 划分为 16 个扇区
    MAX_RANGE = 10.0                  # LiDAR 最大量程(m)
    SECTOR_MAX = 8.0                  # 特征截断距离(m)，用于归一化

    # ---- 神经网络 ----
    INPUT_DIM = NUM_SECTORS + 2       # 16 扇区距离 + 目标方向 sin/cos
    HIDDEN_DIM = 64
    OUTPUT_DIM = 2                    # (v, omega)
    LOAD_WEIGHT = True                # 是否加载预训练权重

    # ---- 控制 ----
    V_MAX = 1.0                       # 最大线速度 m/s（3.0 太快，撞墙反应不过来）
    W_MAX = 1.5                       # 最大角速度 rad/s
    SAFE_DIST = 2.5                   # 安全距离(m)，低于该值触发避障转向（提前避障）
    GOAL_TOL = 0.8                    # 到达目标判定阈值(m)
    CTRL_RATE = 10.0                  # 控制频率 Hz
    YAW_RATE_K = 2.0                  # 目标方向偏航比例系数

    # ---- SLAM / 导航 ----
    ENABLE_RTABMAP = False            # 是否将里程计/点云发布为 ROS 话题供 rtabmap 建图
    MAP_FRAME = "map"
    ODOM_FRAME = "odom"
    BASE_FRAME = "base_link"

    # ---- 训练 ----
    TRAIN_EPOCHS = 60
    TRAIN_LR = 1e-3
    TRAIN_BATCH = 64
    TRAIN_SAMPLES = 4000              # 行为克隆专家数据条数
    SEED = 42
