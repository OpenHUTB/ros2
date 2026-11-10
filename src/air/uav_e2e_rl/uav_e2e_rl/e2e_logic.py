"""uav_e2e_rl 纯逻辑：图像预处理、深度碰撞检测、奖励计算。

不 import rclpy / rospy / cv2，方便 CI 在没 ROS、没 opencv 的环境下单测。
"""
from __future__ import annotations

import numpy as np


DEFAULT_CFG = {
    "img_w": 64,
    "img_h": 48,
    "forward_gain": 0.5,       # 每米前进距离奖励
    "collision_penalty": 5.0,  # 撞一次惩罚
    "yaw_penalty": 0.1,        # 每 rad/s 转向惩罚
    "collision_depth": 0.6,    # 深度图前方最小距离小于这个算撞
    "max_steps": 500,          # 每个 episode 最大步数
}


def preprocess_image(encoding: str, data: bytes, width: int, height: int,
                     img_w: int, img_h: int) -> np.ndarray:
    """把 sensor_msgs/Image 转成 (img_h, img_w) float32 [0,1] 灰度图。

    支持 mono8 / rgb8 / bgr8 / rgba8 / bgra8；不支持的编码抛 ValueError。
    """
    enc = (encoding or "").lower()
    if enc == "mono8":
        gray = np.frombuffer(data, dtype=np.uint8).reshape(height, width)
    elif enc in ("rgb8", "bgr8"):
        arr = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)
        gray = arr.mean(axis=2)
    elif enc in ("rgba8", "bgra8"):
        arr = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 4)
        gray = arr[:, :, :3].mean(axis=2)
    else:
        raise ValueError(f"unsupported encoding: {encoding}")

    if gray.shape != (img_h, img_w):
        # 朴素最近邻缩放（避免依赖 cv2）
        yi = (np.arange(img_h) * gray.shape[0] / img_h).astype(int)
        xi = (np.arange(img_w) * gray.shape[1] / img_w).astype(int)
        gray = gray[np.ix_(yi, xi)]
    return gray.astype(np.float32) / 255.0


def min_depth_in_forward_sector(depth: np.ndarray,
                                v_split: float = 0.5,
                                h_half: float = 0.3) -> float:
    """深度图前向扇区（屏幕下半部中间）的最近距离，米。

    depth: (H, W) float32，单位米。无效值（<=0 或 nan）会被忽略。
    没有有效像素时返回 inf。
    """
    d = np.asarray(depth, dtype=np.float32)
    if d.size == 0:
        return float("inf")
    h, w = d.shape
    v0 = int(h * v_split)
    c = w // 2
    half = int(w * h_half)
    roi = d[v0:, max(0, c - half):min(w, c + half)]
    valid = roi[np.isfinite(roi) & (roi > 0.05)]
    if valid.size == 0:
        return float("inf")
    return float(valid.min())


def compute_reward(forward_dx: float, wz: float, collided: bool, cfg=None) -> float:
    """奖励：前进距离*gain - 碰撞惩罚 - 转向惩罚。

    撞车那一步直接返回 -collision_penalty，不再叠转向惩罚（修 review #4）。
    """
    c = dict(DEFAULT_CFG)
    if cfg:
        c.update(cfg)
    if collided:
        return -float(c["collision_penalty"])
    return (float(c["forward_gain"]) * float(forward_dx)
            - float(c["yaw_penalty"]) * abs(float(wz)))


def is_collision(depth: np.ndarray, threshold: float, **kw) -> bool:
    return min_depth_in_forward_sector(depth, **kw) < float(threshold)


def clip_action(vx: float, wz: float) -> tuple:
    """动作裁剪到 [0,2] m/s 和 [-1,1] rad/s。"""
    return (float(np.clip(vx, 0.0, 2.0)), float(np.clip(wz, -1.0, 1.0)))
