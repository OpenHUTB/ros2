#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""传感器层：RGB / 深度 / 语义分割 / 激光雷达 / IMU，视觉三路共用同一渲染器。

深度单位米、超量程置 0；语义类别按 geom 名字前缀匹配；雷达射线排除自身机体。
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import mujoco
import numpy as np

# 无头机器可用环境变量切换渲染后端（osmesa / egl / glfw / wgl）
os.environ.setdefault("MUJOCO_GL", "wgl" if os.name == "nt" else "glfw")


# --------------------------------------------------------------------------- #
# 语义类别表：geom 名字前缀 → 类别 id
# --------------------------------------------------------------------------- #
# id 0 保留为「背景/未分类」，与无命中区分。
SEMANTIC_CLASSES: Dict[str, int] = {
    "floor": 1, "ground": 1, "terrain": 1,      # 地面
    "wall": 2, "building": 2, "house": 2,       # 建筑/墙
    "tree": 3, "bush": 3, "vegetation": 3,      # 植被
    "obstacle": 4, "box": 4, "barrel": 4,       # 一般障碍物
    "gate": 5, "door": 5,                       # 门（竞速任务）
    "car": 6, "vehicle": 6,                     # 车辆
    "person": 7, "pedestrian": 7,               # 行人
    "drone": 8, "uav": 8, "rotor": 8, "arm": 8, "body": 8, "prop": 8,  # 自身
}


def build_semantic_map(model: mujoco.MjModel) -> Tuple[np.ndarray, Dict[str, int]]:
    """建立「geom id → 语义类别 id」查找表。

    按 geom 名字前缀匹配；未命中者按所属 body 区分「自身」与「障碍物」。
    """
    ngeom = model.ngeom
    table = np.zeros(ngeom, dtype=np.int32)
    inv: Dict[str, int] = {}
    for name, cid in SEMANTIC_CLASSES.items():
        inv.setdefault(name, cid)

    for g in range(ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        cid = 0
        low = name.lower()
        for prefix, c in SEMANTIC_CLASSES.items():
            if low.startswith(prefix) or f"_{prefix}" in low:
                cid = c
                break
        if cid == 0:
            # 前缀未命中时按所属 body 兜底：机体子几何判为「自身」
            body = int(model.geom_bodyid[g])
            bname = (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body) or "").lower()
            if any(k in bname for k in ("base", "rotor")):
                cid = SEMANTIC_CLASSES["drone"]
            else:
                cid = SEMANTIC_CLASSES["obstacle"]
        table[g] = cid
    return table, dict(inv)


# --------------------------------------------------------------------------- #
# 视觉传感器
# --------------------------------------------------------------------------- #
@dataclass
class CameraConfig:
    width: int = 640
    height: int = 480
    name: str = "front_cam"
    # 量程对齐 RealSense D435（0.2~10 m）；MuJoCo 原生返回至远裁剪面（实测约 130 m），
    # 超量程像素置 0
    depth_max_range: float = 10.0
    depth_min_range: float = 0.2
    #: 渲染质量："fast" 关阴影/反射/天空盒（感知用），"full" 全开（演示录制用）。
    #: 实测 RGB 320x240：42 ms(24 Hz) → 6.6 ms(151 Hz)，约 6 倍。
    quality: str = "fast"
    #: 传感器噪声幅度（域随机化 / 鲁棒性对比用）
    rgb_noise: float = 0.0


class VisionSensors:
    """RGB / 深度 / 语义分割三路视觉传感器，共用相机位姿。

    三路图出自同一渲染器与同一次 ``update_scene``，时间同步。
    """

    def __init__(self, model: mujoco.MjModel, cfg: Optional[CameraConfig] = None):
        self.model = model
        self.cfg = cfg or CameraConfig()
        self._renderer: Optional[mujoco.Renderer] = None
        self._mode = "rgb"
        self.sem_table, self.sem_names = build_semantic_map(model)
        self._rng = np.random.default_rng(0)

    def _apply_quality(self, r: mujoco.Renderer) -> None:
        """按 quality 设置渲染开关。阴影占总收益的 85%，反射/天空盒收益小。"""
        fl = r.scene.flags
        S = mujoco.mjtRndFlag.mjRND_SHADOW
        if self.cfg.quality == "fast":
            fl[S] = 0
            fl[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
            fl[mujoco.mjtRndFlag.mjRND_SKYBOX] = 0
        else:
            fl[S] = 1
            fl[mujoco.mjtRndFlag.mjRND_REFLECTION] = 1
            fl[mujoco.mjtRndFlag.mjRND_SKYBOX] = 1

    def _ensure(self, mode: str) -> mujoco.Renderer:
        """切换渲染模式。只改渲染标志位，不 close 重建 Renderer（重建 200~400 ms）。"""
        if self._renderer is None:
            self._renderer = mujoco.Renderer(
                self.model, height=self.cfg.height, width=self.cfg.width
            )
            self._apply_quality(self._renderer)
        if mode != self._mode:
            r = self._renderer
            if mode == "rgb":
                r.disable_depth_rendering()
                r.disable_segmentation_rendering()
            elif mode == "depth":
                r.enable_depth_rendering()
            elif mode == "seg":
                r.enable_segmentation_rendering()
            self._mode = mode
        return self._renderer

    # ------------------------------------------------------------------ RGB
    def rgb(self, data: mujoco.MjData, camera: Optional[str] = None) -> np.ndarray:
        """返回 HxWx3 uint8 RGB 图。"""
        r = self._ensure("rgb")
        r.update_scene(data, camera=camera or self.cfg.name)
        img = r.render().copy()
        if self.cfg.rgb_noise > 0:
            noise = self._rng.normal(0.0, self.cfg.rgb_noise * 255.0, img.shape)
            img = np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)
        return img

    # ----------------------------------------------------------------- 深度
    def depth(self, data: mujoco.MjData,
              camera: Optional[str] = None) -> np.ndarray:
        """返回 HxW float32 深度图，单位米；超出量程的像素为 0。"""
        r = self._ensure("depth")
        r.update_scene(data, camera=camera or self.cfg.name)
        d = r.render().astype(np.float32).copy()
        invalid = (~np.isfinite(d)) | (d > self.cfg.depth_max_range) | (d < self.cfg.depth_min_range)
        d[invalid] = 0.0
        return d

    # ------------------------------------------------------------- 语义分割
    def segmentation(self, data: mujoco.MjData,
                     camera: Optional[str] = None) -> np.ndarray:
        """返回 HxW int32 语义类别图（类别 id 见 SEMANTIC_CLASSES）。

        原始输出 (H, W, 2) 的通道为 (对象 id, 对象类型 mjtObj)；mjOBJ_GEOM == 5，背景为 [-1, -1]。
        """
        r = self._ensure("seg")
        r.update_scene(data, camera=camera or self.cfg.name)
        seg = r.render()
        out = np.zeros(seg.shape[:2], dtype=np.int32)
        is_geom = seg[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM)
        gid = seg[..., 0]
        valid = is_geom & (gid >= 0) & (gid < len(self.sem_table))
        out[valid] = self.sem_table[gid[valid]]
        return out

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass


# --------------------------------------------------------------------------- #
# 激光雷达
# --------------------------------------------------------------------------- #
@dataclass
class LidarConfig:
    n_azimuth: int = 72                  # 水平方向线数
    elevations_deg: Sequence[float] = (-15.0, 0.0, 15.0)   # 垂直层
    max_range: float = 15.0
    min_range: float = 0.3               # 近处截断，抑制自身残影
    azimuth_fov_deg: float = 360.0       # 360 为全向；<360 时以机头为中心
    frame: str = "world"                 # world(ENU) 或 body


class LidarSensor:
    """基于 ``mj_multiRay`` 的激光雷达，一次调用投出全部射线（72 线约 9.7 万次扫描/秒）。

    ``bodyexclude`` 必须指向机体，否则射线命中自身机臂返回接近 0 的距离。
    """

    def __init__(self, model: mujoco.MjModel, cfg: Optional[LidarConfig] = None,
                 exclude_body: str = "base_link", site_name: str = "lidar"):
        self.model = model
        self.cfg = cfg or LidarConfig()
        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if self.site_id < 0:
            raise ValueError(f"模型里找不到雷达安装点 site {site_name!r}")
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, exclude_body)
        self.exclude_body = int(bid)
        self._dirs_local = self._build_dirs()
        self.n_ray = len(self._dirs_local)

    def _build_dirs(self) -> np.ndarray:
        """按方位角 × 俯仰角生成机体系单位方向向量，形状 (n_ray, 3)。"""
        cfg = self.cfg
        if cfg.azimuth_fov_deg >= 360.0:
            az = np.linspace(0.0, 2 * np.pi, cfg.n_azimuth, endpoint=False)
        else:
            half = np.deg2rad(cfg.azimuth_fov_deg) / 2
            az = np.linspace(-half, half, cfg.n_azimuth, endpoint=False)
        el = np.deg2rad(np.asarray(cfg.elevations_deg, dtype=float))
        dirs = []
        for e in el:
            for a in az:
                dirs.append([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
        return np.asarray(dirs, dtype=np.float64)

    def scan(self, data: mujoco.MjData,
             state_pos: Optional[np.ndarray] = None,
             state_mat: Optional[np.ndarray] = None) -> np.ndarray:
        """执行一次扫描，返回 (N,3) 命中点，坐标见 ``cfg.frame``（world(ENU)/body）。

        state_pos/state_mat 为机体位姿（世界位置 + 3x3 旋转矩阵）；省略则用 site 当前位姿（需已 ``mj_forward``）。
        """
        R_site = data.site_xmat[self.site_id].reshape(3, 3).copy()
        p_site = data.site_xpos[self.site_id].copy()

        vec = (R_site @ self._dirs_local.T).T.reshape(-1, 1).copy()   # (3*n,1) 世界系
        n = self.n_ray
        geomid = np.full((n, 1), -1, dtype=np.int32)
        dist = np.full((n, 1), np.inf, dtype=np.float64)

        mujoco.mj_multiRay(
            self.model, data, p_site, vec,
            None, 1, self.exclude_body,
            geomid, dist, None, n, float(self.cfg.max_range),
        )

        hit = (geomid[:, 0] >= 0) & (dist[:, 0] >= self.cfg.min_range)
        hit = hit & (dist[:, 0] <= self.cfg.max_range)
        if not np.any(hit):
            return np.zeros((0, 3), dtype=np.float32)

        d = dist[hit, 0]
        pts_world = p_site[None, :] + self._dirs_local[hit] * d[:, None] @ R_site.T
        if self.cfg.frame == "body":
            if state_mat is None:
                return self._dirs_local[hit].astype(np.float32) * d[:, None].astype(np.float32)
            pts_body = (pts_world - np.asarray(state_pos)) @ np.asarray(state_mat)
            return pts_body.astype(np.float32)
        if state_pos is not None:
            # 相对机体中心输出（扇形直方图使用）
            return (pts_world - np.asarray(state_pos)).astype(np.float32)
        return pts_world.astype(np.float32)

    def fan_histogram(self, data: mujoco.MjData, n_sectors: int = 16,
                      max_range: Optional[float] = None) -> np.ndarray:
        """水平面扇形最小距离直方图（RL 观测用，量纲与上游 uav_ppo_nav 一致）。

        返回长度 n_sectors 的数组，每格为扇区最小命中距离，无命中填 max_range。
        """
        rng = float(max_range if max_range is not None else self.cfg.max_range)
        R_site = data.site_xmat[self.site_id].reshape(3, 3).copy()
        p_site = data.site_xpos[self.site_id].copy()

        az = np.linspace(0.0, 2 * np.pi, n_sectors, endpoint=False)
        dirs = np.stack([np.cos(az), np.sin(az), np.zeros_like(az)], axis=1)
        vec = (R_site @ dirs.T).T.reshape(-1, 1).copy()
        geomid = np.full((n_sectors, 1), -1, dtype=np.int32)
        dist = np.full((n_sectors, 1), np.inf, dtype=np.float64)
        mujoco.mj_multiRay(self.model, data, p_site, vec, None, 1,
                           self.exclude_body, geomid, dist, None, n_sectors, rng)

        out = np.full(n_sectors, rng, dtype=np.float32)
        hit = (geomid[:, 0] >= 0) & (dist[:, 0] >= self.cfg.min_range)
        out[hit] = np.clip(dist[hit, 0], 0.0, rng)
        return out


# --------------------------------------------------------------------------- #
# IMU
# --------------------------------------------------------------------------- #
@dataclass
class ImuReading:
    quat: np.ndarray        # (4,) (w,x,y,z) 机体→世界
    gyro: np.ndarray        # (3,) 机体系角速度 rad/s
    acc: np.ndarray         # (3,) 机体系比力 m/s^2（含重力，真实 IMU 语义）
    vel_body: np.ndarray    # (3,) 机体系线速度 m/s（真值，供评估用）


class ImuSensor:
    """从模型的 ``<sensor>`` 段读取 IMU。

    支持偏置与高斯噪声；噪声按 rate_hz 采样并保持，模拟真实 IMU 输出频率。
    """

    def __init__(self, model: mujoco.MjModel,
                 gyro_noise: float = 0.0, acc_noise: float = 0.0,
                 gyro_bias: Optional[Sequence[float]] = None,
                 acc_bias: Optional[Sequence[float]] = None,
                 rate_hz: float = 200.0, seed: int = 0):
        def sid(n):
            i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, n)
            if i < 0:
                raise ValueError(f"模型里找不到传感器 {n!r}")
            return i, int(model.sensor_adr[i]), int(model.sensor_dim[i])

        self._q = sid("imu_quat")
        self._g = sid("imu_gyro")
        self._a = sid("imu_acc")
        self._v = sid("imu_vel")
        self.gyro_noise = float(gyro_noise)
        self.acc_noise = float(acc_noise)
        self.gyro_bias = np.asarray(gyro_bias if gyro_bias is not None else np.zeros(3), float)
        self.acc_bias = np.asarray(acc_bias if acc_bias is not None else np.zeros(3), float)
        self.rate_hz = float(rate_hz)
        self._rng = np.random.default_rng(seed)
        self._held_gyro = np.zeros(3)
        self._held_acc = np.zeros(3)
        self._t_next = 0.0
        self._hold_dt = 1.0 / max(self.rate_hz, 1e-6)

    def _get(self, data: mujoco.MjData, spec) -> np.ndarray:
        i, adr, dim = spec
        return np.array(data.sensordata[adr:adr + dim], dtype=float)

    def read(self, data: mujoco.MjData, t: float) -> ImuReading:
        if t >= self._t_next:
            self._t_next = t + self._hold_dt
            self._held_gyro = self._rng.normal(0.0, self.gyro_noise, 3) if self.gyro_noise > 0 else np.zeros(3)
            self._held_acc = self._rng.normal(0.0, self.acc_noise, 3) if self.acc_noise > 0 else np.zeros(3)
        quat = self._get(data, self._q)
        gyro = self._get(data, self._g) + self.gyro_bias + self._held_gyro
        acc = self._get(data, self._a) + self.acc_bias + self._held_acc
        vel = self._get(data, self._v)
        return ImuReading(quat=quat, gyro=gyro, acc=acc, vel_body=vel)

    def reset(self) -> None:
        self._t_next = 0.0
        self._held_gyro[:] = 0.0
        self._held_acc[:] = 0.0
