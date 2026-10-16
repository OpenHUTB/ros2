#!/usr/bin/env python3
"""collect_data.py —— 端到端数据采集（深度相机图像 + 专家控制指令）

流程：
    1) 连接 AirSim，起飞悬停
    2) 自动探索飞行：每帧获取前视深度图，用专家规则生成 (v, w) 标签
    3) 保存 data/e2e_data.npz (images, actions)

专家规则：
    - 深度图分左/中/右三区，取平均归一化距离
    - 中心近  -> 减速
    - 左边近  -> 右转；右边近 -> 左转
    - 都远    -> 全速直飞

运行： python3 collect_data.py
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from config import Config


def get_depth_image(client, cfg):
    """获取前视深度图 -> (IMG_H, IMG_W) 归一化 [0,1] float32。"""
    import airsim
    resp = client.simGetImages([
        airsim.ImageRequest(cfg.CAMERA_NAME, airsim.ImageType.DepthPerspective,
                            True, False)])   # pixels_as_float=True 返回 float 深度
    if not resp or resp[0].image_data_float is None or len(resp[0].image_data_float) == 0:
        return None
    depth = airsim.list_to_2d_float_array(resp[0].image_data_float,
                                          resp[0].width, resp[0].height)
    depth = depth / 100.0   # AirSim 深度单位是厘米(cm)，转米(m)
    depth = np.clip(depth, 0.0, cfg.MAX_RANGE) / cfg.MAX_RANGE
    # 降采样到目标尺寸（无 cv2 依赖，用 strided + 中心裁剪）
    h, w = depth.shape
    sh = max(1, h // cfg.IMG_H)
    sw = max(1, w // cfg.IMG_W)
    img = depth[::sh, ::sw]
    img = img[:cfg.IMG_H, :cfg.IMG_W]
    # 若尺寸不足，用 0 填充（远距离）
    out = np.zeros((cfg.IMG_H, cfg.IMG_W), dtype=np.float32)
    out[:img.shape[0], :img.shape[1]] = img
    return out


def expert_action(depth_img, cfg):
    """专家规则：左/中/右区域平均距离 -> (v_norm, w_norm)。"""
    h, w = depth_img.shape
    left = float(depth_img[:, : w // 3].mean())
    center = float(depth_img[:, w // 3 : 2 * w // 3].mean())
    right = float(depth_img[:, 2 * w // 3 :].mean())

    v = 1.0 if center > 0.5 else 0.3
    if left < 0.30 and left < right - 0.05:
        w = 1.0      # 左边近 -> 右转
    elif right < 0.30 and right < left - 0.05:
        w = -1.0     # 右边近 -> 左转
    else:
        w = 0.0
    return v, w


def main():
    cfg = Config()
    import airsim

    print("=" * 56)
    print(" task4_e2e : 数据采集（深度图 -> 专家动作）")
    print(f" 采集时长={cfg.COLLECT_SECONDS}s  相机={cfg.CAMERA_NAME}")
    print("=" * 56)

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
    print("[AirSim] 连接成功，无人机已爬升至 5m 悬停，开始采集 ...")

    os.makedirs(cfg.DATASET_DIR, exist_ok=True)
    imgs, acts = [], []
    rng = np.random.default_rng(cfg.SEED)
    period = 1.0 / cfg.CTRL_RATE
    start = time.time()

    while time.time() - start < cfg.COLLECT_SECONDS:
        img = get_depth_image(client, cfg)
        if img is None:
            time.sleep(0.1)
            continue
        v, w = expert_action(img, cfg)
        # 探索噪声，增强数据多样性
        w = float(np.clip(w + rng.uniform(-0.2, 0.2), -1.0, 1.0))
        v = float(np.clip(v + rng.uniform(-0.1, 0.1), 0.0, 1.0))

        imgs.append(img)
        acts.append([v, w])

        client.moveByVelocityBodyFrameAsync(
            v * cfg.V_MAX, 0.0, 0.0, period,
            yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=w * cfg.W_MAX),
            vehicle_name=cfg.VEHICLE_NAME).join()

        if len(imgs) % 50 == 0:
            print(f"  已采集 {len(imgs)} 帧 ({time.time()-start:.0f}s)")

    np.savez(os.path.join(cfg.DATASET_DIR, "e2e_data.npz"),
             images=np.array(imgs, dtype=np.float32),
             actions=np.array(acts, dtype=np.float32))
    print(f"[完成] 采集 {len(imgs)} 帧 -> {os.path.join(cfg.DATASET_DIR, 'e2e_data.npz')}")

    client.landAsync().join()
    client.armDisarm(False, cfg.VEHICLE_NAME)
    client.enableApiControl(False, cfg.VEHICLE_NAME)
    return 0


if __name__ == "__main__":
    sys.exit(main())
