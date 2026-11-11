"""针孔相机 + 解析光线投射渲染（RGB-D）。

相机装在车头，光轴与车头方向一致，图像左边界对应车体左侧（+y 方向）。
渲染分两步：先对每列投一条水平射线拿到命中距离与类别，再按透视关系
把命中物"竖"起来铺成图像列。同一列的所有像素共享一个深度值，
等价于一个 160 线的单行深度传感器 —— 这正是 RGB-D 相机在本仿真里的角色。
"""

import math

import numpy as np

from . import params as P
from .world import PALETTE

CEIL_COLOR = np.array([0.075, 0.078, 0.092])
FLOOR_COLOR = np.array([0.30, 0.30, 0.33])
WALL_COLOR = np.array([0.66, 0.67, 0.70])
DYN_COLOR = np.array([0.28, 0.24, 0.20])

_DYN_H = 0.55   # 动态障碍高度（米），与路标接近


def col_angles():
    """每列对应的车体坐标系方位角（左正右负）。"""
    fov = P.CAM_FOV
    return fov / 2.0 - np.arange(P.CAM_W) * fov / (P.CAM_W - 1)


def render(world, pose, rng=None, night=False, slip_scale=1.0):
    """渲染一帧。

    返回
    ----
    rgb : uint8 (H, W, 3)
    depth : float32 (H, W)，单位米，超过量程处为 CAM_RANGE_MAX
    """
    rng = np.random.default_rng() if rng is None else rng
    x, y, th = float(pose[0]), float(pose[1]), float(pose[2])
    ang = th + col_angles()

    dist, hid = world.raycast((x, y), ang)

    n_rect = len(world.rects)
    n_lm = len(world.landmarks)

    # 每列的顶部/底部像素行
    h_px = np.where(hid <= n_rect, P.WALL_H,
                    np.where(hid <= n_rect + n_lm, P.LM_HEIGHT, _DYN_H))
    d = np.maximum(dist, 0.05)
    cy = P.CAM_H / 2.0
    y_top = cy - (h_px - P.CAM_H_MOUNT) * P.CAM_F / d
    y_bot = cy + P.CAM_H_MOUNT * P.CAM_F / d

    # 每列颜色：路标取调色板，墙/动态取固定色，未命中的列不参与盖色
    col_color = np.tile(WALL_COLOR, (P.CAM_W, 1))
    is_lm = (hid > n_rect) & (hid <= n_rect + n_lm)
    if is_lm.any():
        col_color[is_lm] = PALETTE[hid[is_lm] - n_rect - 1]
    is_dyn = hid > (n_rect + n_lm)
    if is_dyn.any():
        col_color[is_dyn] = DYN_COLOR

    hit = hid > 0
    # 距离衰减：远处更暗，模拟光照/对比度下降
    fade = np.clip(1.0 - 0.052 * dist, 0.30, 1.0)[:, None]
    if night:
        fade = fade * 0.68
    col_color = col_color * fade

    rows = np.arange(P.CAM_H)[:, None]
    mask = hit[None, :] & (rows >= y_top[None, :]) & (rows <= y_bot[None, :])

    # 背景：上半天花板、下半地板，带轻微渐变
    t = (np.arange(P.CAM_H) / (P.CAM_H - 1.0))[:, None, None]
    bg = np.where(t < 0.5, CEIL_COLOR[None, None, :], FLOOR_COLOR[None, None, :])
    bg = np.repeat(bg, P.CAM_W, axis=1) * (1.0 if not night else 0.68)

    img = np.where(mask[:, :, None], col_color[None, :, :], bg)

    # 相机噪声：读出噪声 + 少量乘性噪声
    sigma = 0.030 if night else 0.018
    img = img + rng.normal(0.0, sigma, img.shape) * slip_scale
    img = np.clip(img, 0.0, 1.0)

    depth = np.repeat(dist[None, :], P.CAM_H, axis=0).astype(np.float32)
    # 深度噪声：随距离平方增长（视差误差的典型形态）
    dn = rng.normal(0.0, 0.012 + 0.010 * dist ** 1.5, dist.shape) * slip_scale
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


def landmark_heights(rgb, depth):
    """占位：保留单目测距的接口（由像素高度反推距离），供文档对比说明。"""
    return None
