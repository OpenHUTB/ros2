"""前视相机 + 解析光线投射渲染（RGB-D）。

相机装在 ROV 头部，光轴水平、与艏向一致。渲染分两步：先对每列投一条
水平射线拿到命中距离与类别（应答器 / 管道），再按透视关系把命中物
按已知高度"立"起来铺成图像列。同一列的所有像素共享一个深度值，
等价于一个 128 线的单行深度传感器 —— 这是本仿真里 RGB-D 的角色。

水下特色：距离衰减比空气里快得多（光被吸收与散射），浑水场景再加
一层向水色收敛的雾 + 强读出噪声 —— 这正是几何阈值检测会失效、
需要学习式感知的场景。
"""

import math

import numpy as np

from . import params as P
from .world import PALETTE, PIPE_COLOR

WATER_TOP = np.array([0.05, 0.14, 0.19])     # 远处/上层水色
WATER_BOT = np.array([0.10, 0.24, 0.28])     # 近处/下层水色
SEDIMENT = np.array([0.34, 0.31, 0.26])      # 海床沉积物

_LM_H = P.LM_HEIGHT
_PIPE_H = P.PIPE_HEIGHT


def col_angles():
    """每列对应的车体坐标系方位角（左正右负）。"""
    fov = P.CAM_FOV
    return fov / 2.0 - np.arange(P.CAM_W) * fov / (P.CAM_W - 1)


def render(world, pose, rng=None, turbid=None):
    """渲染一帧。

    返回
    ----
    rgb : uint8 (H, W, 3)
    depth : float32 (H, W)，单位米，超过量程处为 CAM_RANGE_MAX
    """
    rng = np.random.default_rng() if rng is None else rng
    turbid = world.turbid if turbid is None else bool(turbid)
    x, y, th = float(pose[0]), float(pose[1]), float(pose[2])
    ang = th + col_angles()

    dist, hid = world.raycast((x, y), ang)

    n_lm = len(world.landmarks)
    is_lm = (hid >= 1) & (hid <= n_lm)
    is_pipe = hid == (n_lm + 1)

    d = np.maximum(dist, 0.05)

    # 每列物体的竖直像素范围：应答器立杆、半埋管道顶
    cy = P.CAM_H * 0.62                      # 海平面方向的地平线基准行
    h_px = np.where(is_lm, _LM_H, np.where(is_pipe, _PIPE_H, 0.0))
    y_top = cy - (h_px - P.CAM_ALT) * P.CAM_F / d
    y_bot = cy + P.CAM_ALT * P.CAM_F / d

    # 每列颜色
    col_color = np.tile(PIPE_COLOR, (P.CAM_W, 1))
    if is_lm.any():
        col_color[is_lm] = PALETTE[np.clip(hid[is_lm] - 1, 0, len(PALETTE) - 1)]

    # 水下光衰减：比空气强得多；浑水更狠
    fade = np.clip(np.exp(-0.16 * d if not turbid else -0.34 * d), 0.10, 1.0)[:, None]
    col_color = col_color * fade

    rows = np.arange(P.CAM_H)[:, None]
    hit = (is_lm | is_pipe)
    mask = hit[None, :] & (rows >= y_top[None, :]) & (rows <= y_bot[None, :])

    # 背景：上层水色渐变 + 底部海床沉积物带（地平线随距离上抬的近似）
    t = (rows / (P.CAM_H - 1.0))
    water = WATER_TOP[None, None, :] * (1 - t[:, :, None]) + \
        WATER_BOT[None, None, :] * t[:, :, None]
    seabed_row = cy + P.CAM_ALT * P.CAM_F / P.CAM_RANGE_MAX
    bed = t > min(0.9, seabed_row / P.CAM_H + 0.30)
    bg = np.where(bed[:, :, None], SEDIMENT[None, None, :], water)
    bg = np.repeat(bg, P.CAM_W, axis=1)

    img = np.where(mask[:, :, None], col_color[None, :, :], bg)

    # 浑水雾化：向水色收敛，距离越远雾越重
    if turbid:
        fog = np.clip(1.0 - np.exp(-0.30 * d), 0.0, 0.92)[:, None]
        img = img * (1 - fog) + WATER_BOT[None, None, :] * fog

    # 读出噪声
    sigma = 0.055 if turbid else 0.020
    img = img + rng.normal(0.0, sigma, img.shape)
    img = np.clip(img, 0.0, 1.0)

    depth = np.repeat(dist[None, :], P.CAM_H, axis=0).astype(np.float32)
    # 深度噪声：随距离增长，浑水下声学/光学测距都更差
    dn = rng.normal(0.0, (0.012 + 0.010 * dist ** 1.5) * (1.8 if turbid else 1.0),
                    dist.shape)
    depth = np.clip(depth + dn[None, :], 0.02, P.CAM_RANGE_MAX).astype(np.float32)

    return (img * 255.0).astype(np.uint8), depth


def to_gray(rgb):
    """RGB → 灰度（ITU-R BT.601），神经网络输入用。"""
    f = np.array([0.299, 0.587, 0.114], dtype=np.float32)
    return (rgb.astype(np.float32) / 255.0) @ f


def resize_gray(gray, out_w, out_h):
    """面积平均下采样，够用且不引入额外依赖。"""
    h, w = gray.shape
    fy = h // out_h
    fx = w // out_w
    g = gray[:out_h * fy, :out_w * fx]
    return g.reshape(out_h, fy, out_w, fx).mean(axis=(1, 3))
