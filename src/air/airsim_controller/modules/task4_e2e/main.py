#!/usr/bin/env python3
"""main.py —— 端到端 CNN 推理飞行（深度图 -> 速度指令）

流程：
    1) 加载训练好的 E2ECNN 权重 (data/e2e_cnn.pt)
    2) 起飞后爬升到 5m，循环获取前视深度图 -> CNN 前向 -> (v, w) -> 机体速度控制
    3) 安全兜底：前方深度均值过低时减速悬停，防止碰撞

运行： python3 main.py
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from config import Config

try:
    import torch
    import torch.nn as nn
    _TORCH_OK = True
except ImportError:  # pragma: no cover
    torch = None
    nn = None
    _TORCH_OK = False

from collect_data import get_depth_image


class E2ECNN(nn.Module):
    """与 train.py 相同的 CNN 结构。"""

    def __init__(self, in_h=64, in_w=64):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(1, 16, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),
            nn.Conv2d(16, 32, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),
        )
        flat = 32 * (in_h // 4) * (in_w // 4)
        self.head = nn.Sequential(
            nn.Linear(flat, 128), nn.ReLU(),
            nn.Linear(128, 2), nn.Tanh(),
        )

    def forward(self, x):
        return self.head(self.features(x).flatten(1))


def make_control(out, cfg):
    """网络输出 -> 实际速度指令。"""
    v_norm, w_norm = float(out[0]), float(out[1])
    v = (v_norm + 1.0) / 2.0 * cfg.V_MAX      # [0, V_MAX]
    w = w_norm * cfg.W_MAX                     # [-W_MAX, W_MAX]
    return v, w


def safe_override(v, w, depth_img, cfg):
    """安全兜底：前方整体过近时减速，极近时悬停。"""
    center_mean = float(depth_img.mean())
    if center_mean < cfg.HOVER_RANGE * 0.5:
        v = 0.0
        w = 0.0
    elif center_mean < cfg.HOVER_RANGE:
        v = min(v, 0.3 * cfg.V_MAX)
    return v, w


def main():
    cfg = Config()
    print("=" * 56)
    print(" task4_e2e : 端到端 CNN 推理飞行")
    print(f" 输入 64x64 深度图 -> CNN -> (v, w)")
    print("=" * 56)

    if not _TORCH_OK:
        print("[错误] 需要 PyTorch：pip install torch")
        sys.exit(1)

    import airsim

    # 1. 加载网络
    if not os.path.exists(cfg.WEIGHT_PATH):
        print(f"[错误] 未找到权重 {cfg.WEIGHT_PATH}")
        print("       请先运行: python3 collect_data.py && python3 train.py")
        sys.exit(1)
    model = E2ECNN(cfg.IMG_H, cfg.IMG_W)
    model.load_state_dict(torch.load(cfg.WEIGHT_PATH, map_location="cpu"))
    model.eval()
    print(f"[Net] 权重加载成功: {cfg.WEIGHT_PATH}")

    # 2. 连接 AirSim
    client = airsim.MultirotorClient(ip=cfg.AIRSIM_IP, port=cfg.AIRSIM_PORT)
    client.confirmConnection()
    client.enableApiControl(True, cfg.VEHICLE_NAME)
    client.armDisarm(True, cfg.VEHICLE_NAME)
    client.takeoffAsync(2.0, cfg.VEHICLE_NAME).join()
    client.hoverAsync().join()
    # 爬升到 5 米高度，避免贴地飞行（NED: z 向下为负，z=-5 即 5m 高）
    pos = client.getMultirotorState().kinematics_estimated.position
    client.moveToPositionAsync(pos.x_val, pos.y_val, -5.0, 3.0,
                               vehicle_name=cfg.VEHICLE_NAME).join()
    client.hoverAsync().join()
    print("[AirSim] 连接成功，无人机已爬升至 5m 悬停，开始端到端推理飞行 ...")

    # 3. 主控制循环（自由飞行，Ctrl+C 停止）
    period = 1.0 / cfg.CTRL_RATE
    start_t = time.time()
    min_obs = float("inf")
    step = 0
    try:
        while True:
            t0 = time.time()

            img = get_depth_image(client, cfg)
            if img is None:
                time.sleep(0.1)
                continue
            min_obs = min(min_obs, float(img.mean()) * cfg.MAX_RANGE)

            x = torch.from_numpy(img.reshape(1, 1, cfg.IMG_H, cfg.IMG_W).astype(np.float32))
            with torch.no_grad():
                out = model(x).numpy().ravel()
            v, w = make_control(out, cfg)
            v, w = safe_override(v, w, img, cfg)

            client.moveByVelocityBodyFrameAsync(
                v, 0.0, 0.0, period,
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=w),
                vehicle_name=cfg.VEHICLE_NAME).join()

            step += 1
            if step % 20 == 0:
                print(f"[Step {step}] 前方平均距离 {img.mean()*cfg.MAX_RANGE:.2f}m | "
                      f"v={v:.2f} w={w:.2f}")

    except KeyboardInterrupt:
        print("\n[中断] 用户终止推理飞行。")
    finally:
        client.hoverAsync().join()
        client.landAsync().join()
        client.armDisarm(False, cfg.VEHICLE_NAME)
        client.enableApiControl(False, cfg.VEHICLE_NAME)

    elapsed = time.time() - start_t
    print("\n===== 任务4 性能评价 =====")
    print(f"  飞行时长      : {elapsed:.1f} s")
    print(f"  前方最近距离  : {min_obs:.2f} m")
    print("==========================")
    return 0


if __name__ == "__main__":
    sys.exit(main())
