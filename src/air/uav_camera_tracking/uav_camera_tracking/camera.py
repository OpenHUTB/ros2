# -*- coding: utf-8 -*-
"""camera.py —— 机载下视相机的针孔投影与"合成图像"渲染。

为什么要自己渲染图像：本模块的评价对象是**神经网络感知**，必须给它输入
真正的像素数组(而不是直接喂几何坐标)，否则"感知网络"就成了摆设。
这里用光线投射渲染一个极简场景(棋盘地面 + 目标方块 + 干扰方块 + 遮挡墙)，
纯 numpy 向量化实现，单帧 160x120 约 2~4 ms，便于采集上万帧训练数据。
"""
import numpy as np

from . import params as P


class Camera:
    """针孔相机：参数固定，提供投影、渲染与真值标注。"""

    def __init__(self, width=P.IMG_W, height=P.IMG_H,
                 fov_deg=P.FOV_DEG, pitch_deg=P.PITCH_DEG):
        self.w = int(width)
        self.h = int(height)
        self.f_px = (self.w / 2.0) / np.tan(np.deg2rad(fov_deg) / 2.0)
        self.cx = (self.w - 1) / 2.0
        self.cy = (self.h - 1) / 2.0
        self.alpha = np.deg2rad(pitch_deg)

    # ---------- 坐标系 ----------
    def axes(self, yaw):
        """返回相机三轴在世界系的方向：前向 f、右向 r、下向 d。

        约定：f 指向光轴(前下方)，r 指向图像右侧，d 指向图像下方。
        对下俯角 α，f=(cosαcosψ, cosαsinψ, -sinα)，d=(-sinαcosψ, -sinαsinψ, -cosα)，
        这样"目标在光轴下方"就对应像素 v 增大，与真实相机一致。
        """
        ca, sa = np.cos(self.alpha), np.sin(self.alpha)
        cy, sy = np.cos(yaw), np.sin(yaw)
        f = np.array([ca * cy, ca * sy, -sa])
        r = np.array([-sy, cy, 0.0])
        d = np.array([-sa * cy, -sa * sy, -ca])
        return f, r, d

    def project(self, cam_pos, yaw, pts):
        """世界点(...,3) -> 像素坐标(...,2) 与深度(...)。"""
        pts = np.asarray(pts, dtype=float)
        f, r, d = self.axes(yaw)
        v = pts - cam_pos
        xc = v @ f
        yc = v @ r
        zc = v @ d
        safe = np.where(np.abs(xc) < 1e-6, 1e-6, xc)
        u = self.cx + self.f_px * yc / safe
        vv = self.cy + self.f_px * zc / safe
        return np.stack([u, vv], axis=-1), xc

    # ---------- 渲染 ----------
    def render(self, cam_pos, yaw, target_xy, distractors=(), walls=(),
               rng=None, noise=0.02):
        """渲染一帧 RGB 图像(float32, 0~1)。

        target_xy: 目标中心 (x, y)，边长 P.TARGET_SIZE；
        distractors: 干扰方块中心列表；
        walls: 遮挡柱/墙列表，每个为 (cx, cy, half_x, half_y, half_z)。
        """
        f, r, d = self.axes(yaw)
        us = (np.arange(self.w) + 0.5 - self.cx) / self.f_px
        vs = (np.arange(self.h) + 0.5 - self.cy) / self.f_px
        # dirs[v, u, :]
        dirs = (f[None, None, :]
                + us[None, :, None] * r[None, None, :]
                + vs[:, None, None] * d[None, None, :])

        dz = dirs[:, :, 2]
        # 地面 z=0 求交：t = -Cz / dz
        with np.errstate(divide='ignore', invalid='ignore'):
            t_ground = np.where(dz < -1e-6, -cam_pos[2] / dz, np.inf)
        gx = cam_pos[0] + t_ground * dirs[:, :, 0]
        gy = cam_pos[1] + t_ground * dirs[:, :, 1]

        img = np.zeros((self.h, self.w, 3), dtype=np.float32)
        # 天空(射线朝上或无地面交点)：越接近竖直向上越亮
        sky = 0.55 + 0.30 * np.clip(dz, 0.0, 1.0)
        img[:, :, 0] = sky * 0.75
        img[:, :, 1] = sky * 0.85
        img[:, :, 2] = sky * 0.95

        # 地面棋盘格(每 2 m 一格)
        chk = ((np.floor(np.clip(gx, -1e4, 1e4) / 2.0)
                + np.floor(np.clip(gy, -1e4, 1e4) / 2.0)) % 2)
        ground_val = np.where(chk > 0.5, 0.50, 0.40)
        shade = np.clip(1.0 - 0.012 * np.where(np.isfinite(t_ground),
                                              t_ground, 0.0), 0.55, 1.0)
        gmask = np.isfinite(t_ground)
        for c, k in enumerate((0.92, 1.0, 0.92)):   # 略偏绿的地面
            img[:, :, c] = np.where(gmask, ground_val * shade * k, img[:, :, c])

        # 干扰方块(蓝色)与目标(红色)
        def _paint(center_xy, half, color):
            m = (np.abs(gx - center_xy[0]) <= half) & (np.abs(gy - center_xy[1]) <= half) & gmask
            for c in range(3):
                img[:, :, c] = np.where(m, color[c] * shade, img[:, :, c])

        for cxy in distractors:
            _paint(cxy, P.DISTRACTOR_SIZE / 2.0, (0.20, 0.35, 0.85))
        _paint(target_xy, P.TARGET_SIZE / 2.0, (0.88, 0.16, 0.12))

        # 遮挡柱/墙：与线段 [cam, ground_point] 做 slab 相交
        for wall in walls:
            hit = self._box_hit(cam_pos, gx, gy, wall)
            for c, k in enumerate((0.62, 0.60, 0.56)):
                img[:, :, c] = np.where(hit, k, img[:, :, c])

        # 传感器噪声
        if noise > 0:
            rng = np.random.default_rng(0) if rng is None else rng
            img = img + rng.normal(0.0, noise, img.shape).astype(np.float32)
        return np.clip(img, 0.0, 1.0)

    @staticmethod
    def _box_hit(cam_pos, gx, gy, wall):
        """射线与轴对齐长方体是否相交(且交点在到达地面前)。"""
        cx, cy, hx, hy, hz = wall
        lo = np.array([cx - hx, cy - hy, 0.0])
        hi = np.array([cx + hx, cy + hy, 2.0 * hz])
        # 目标点(地面交点)；无交点时用远点，避免 inf 传播
        px = np.where(np.isfinite(gx), gx, cam_pos[0] + 1e3)
        py = np.where(np.isfinite(gy), gy, cam_pos[1] + 1e3)
        pz = 0.0
        dvec = np.stack([px - cam_pos[0], py - cam_pos[1],
                         np.zeros_like(px) + (pz - cam_pos[2])], axis=-1)
        tmin = np.zeros(px.shape)
        tmax = np.ones(px.shape)
        origin = cam_pos
        for i in range(3):
            di = dvec[:, :, i]
            oi = origin[i]
            with np.errstate(divide='ignore', invalid='ignore'):
                t1 = (lo[i] - oi) / di
                t2 = (hi[i] - oi) / di
            tlo = np.minimum(t1, t2)
            thi = np.maximum(t1, t2)
            parallel = np.abs(di) < 1e-9
            inside = (oi >= lo[i]) & (oi <= hi[i])
            tlo = np.where(parallel, np.where(inside, -np.inf, np.inf), tlo)
            thi = np.where(parallel, np.where(inside, np.inf, -np.inf), thi)
            tmin = np.maximum(tmin, tlo)
            tmax = np.minimum(tmax, thi)
        return (tmin <= tmax) & (tmax > 0.0) & (tmin < 1.0)

    # ---------- 真值标注 ----------
    def observe(self, cam_pos, yaw, target_xy, walls=()):
        """返回目标真值：中心像素、是否可见、包围框宽度(像素)。"""
        hs = P.TARGET_SIZE / 2.0
        tx, ty = float(target_xy[0]), float(target_xy[1])
        center = np.array([tx, ty, 0.03])
        uv, depth = self.project(cam_pos, yaw, center)
        u, v = float(uv[0]), float(uv[1])
        corners = np.array([
            [tx - hs, ty - hs, 0.03], [tx + hs, ty - hs, 0.03],
            [tx + hs, ty + hs, 0.03], [tx - hs, ty + hs, 0.03],
        ])
        cuv, _ = self.project(cam_pos, yaw, corners)
        bw = float(np.max(cuv[:, 0]) - np.min(cuv[:, 0]))
        in_fov = (depth > 0.3 and -0.1 * self.w <= u <= 1.1 * self.w
                  and -0.1 * self.h <= v <= 1.1 * self.h)
        occluded = False
        for wall in walls:
            gx = np.array([[tx]])
            gy = np.array([[ty]])
            if bool(self._box_hit(cam_pos, gx, gy, wall)[0, 0]):
                occluded = True
                break
        return {
            'u': u, 'v': v, 'width_px': bw, 'visible': bool(in_fov and not occluded),
            'in_fov': bool(in_fov), 'occluded': occluded, 'depth': float(depth),
        }

    # ---------- 网络输入 ----------
    @staticmethod
    def color_opponent(img):
        """颜色对立通道：红-绿、红-蓝、灰度。

        直接对 RGB 做均值池化时，小目标的色差会被地面颜色稀释；先做这一步
        固定的颜色变换（等价于传统流程里的 HSV 阈值前置），把"红色比周围
        更红多少"显式表达出来，网络的定位难度才和真实任务相当。
        """
        r, g, b = img[:, :, 0], img[:, :, 1], img[:, :, 2]
        d1 = np.clip(r - g, 0.0, 1.0)
        d2 = np.clip(r - b, 0.0, 1.0)
        gray = (r + g + b) / 3.0
        return np.stack([d1, d2, gray], axis=-1).astype(np.float32)

    def downsample(self, img, ds=P.DS):
        """颜色对立通道 -> 均值池化成 ds x ds x 3 的小图。"""
        feat = self.color_opponent(img)
        h, w = feat.shape[0], feat.shape[1]
        bh, bw = h // ds, w // ds
        crop = feat[:bh * ds, :bw * ds]
        small = crop.reshape(ds, bh, ds, bw, 3).mean(axis=(1, 3))
        return small.astype(np.float32)
