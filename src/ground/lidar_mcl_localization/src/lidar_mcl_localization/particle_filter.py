# -*- coding: utf-8 -*-
"""粒子滤波本体：预测 → 加权 → 重采样。

实现要点
--------
* **初始分布**：在给定初始位姿周围按高斯撒点（真实系统里通常也这么初始化，
  或在地图上均匀撒点做"全局定位"——本模块支持 `global_localization=True`）。
* **低方差重采样（low-variance sampling）**：比轮盘赌的方差小，
  同样的粒子数下估计更稳，已由 Thrun 等的定理保证。
* **有效粒子数 Neff**：只有当 Neff < ratio·N 时才重采样。
  每帧都重采样会加速粒子退化（叫 "particle depletion"），因为重采样会丢掉多样性。
* **位姿估计**：位置取加权平均，航向取**圆周平均**——普通平均在 ±π 附近会错。
"""

import math

import numpy as np

from .geometry import weighted_pose, wrap_angle


class ParticleFilter(object):

    def __init__(self, num_particles=500, init_pose=(0.0, 0.0, 0.0),
                 init_std=(0.30, 0.30, 0.15), rng=None, global_localization=False,
                 occ_map=None):
        self.num_particles = int(num_particles)
        self.rng = rng if rng is not None else np.random.RandomState(0)
        self.particles = np.zeros((self.num_particles, 3), dtype=np.float64)
        self.weights = np.ones(self.num_particles, dtype=np.float64) / self.num_particles
        self.resample_count = 0

        if global_localization and occ_map is not None:
            self.initialize_global(occ_map)
        else:
            self.initialize_gaussian(init_pose, init_std)

    # ------------------------------------------------------------ 初始化
    def initialize_gaussian(self, pose, std):
        n = self.num_particles
        self.particles[:, 0] = self.rng.normal(pose[0], std[0], n)
        self.particles[:, 1] = self.rng.normal(pose[1], std[1], n)
        self.particles[:, 2] = self.rng.normal(pose[2], std[2], n)
        self.particles[:, 2] = np.array([wrap_angle(v) for v in self.particles[:, 2]])
        self.weights[:] = 1.0 / n

    def initialize_global(self, occ_map):
        """在地图的所有空闲格里均匀撒点（全局定位 / 绑架问题）。"""
        free_rows, free_cols = np.nonzero(occ_map.grid == 254)
        if free_rows.size == 0:
            raise ValueError("地图里没有空闲格，无法全局初始化")
        n = self.num_particles
        pick = self.rng.randint(0, free_rows.size, n)
        xs, ys = occ_map.grid_to_world(free_cols[pick], free_rows[pick])
        self.particles[:, 0] = xs
        self.particles[:, 1] = ys
        self.particles[:, 2] = self.rng.uniform(-math.pi, math.pi, n)
        self.weights[:] = 1.0 / n

    # -------------------------------------------------------------- 预测
    def predict(self, motion_model, delta, corrector=None, context=None):
        self.particles = motion_model.sample_with_correction(
            self.particles, delta, self.rng, corrector=corrector, context=context)

    # -------------------------------------------------------------- 更新
    def update(self, sensor_model, angles, ranges, occ_map, max_range):
        if angles.size == 0:
            return
        self.weights = sensor_model.weights(self.particles, angles, ranges, occ_map, max_range)

    # ------------------------------------------------------------ 重采样
    def neff(self):
        return 1.0 / float(np.sum(self.weights ** 2))

    def maybe_resample(self, ratio=0.5):
        if self.neff() < ratio * self.num_particles:
            self.resample()
            return True
        return False

    def resample(self):
        """低方差（系统）重采样。"""
        n = self.num_particles
        positions = (self.rng.uniform(0.0, 1.0) + np.arange(n)) / n
        cumulative = np.cumsum(self.weights)
        cumulative[-1] = 1.0

        idx = np.searchsorted(cumulative, positions)
        idx = np.clip(idx, 0, n - 1)

        self.particles = self.particles[idx].copy()
        self.weights = np.ones(n, dtype=np.float64) / n
        self.resample_count += 1

    # -------------------------------------------------------------- 输出
    def estimate(self):
        return weighted_pose(self.particles, self.weights)

    def spread(self):
        """粒子散布（米 / 弧度），用于判断当前置信度。"""
        x, y, yaw = self.estimate()
        dx = self.particles[:, 0] - x
        dy = self.particles[:, 1] - y
        pos_std = float(np.sqrt(np.mean(dx * dx + dy * dy)))
        dth = np.array([wrap_angle(v - yaw) for v in self.particles[:, 2]])
        yaw_std = float(np.sqrt(np.mean(dth * dth)))
        return pos_std, yaw_std

    def covariance(self):
        """位置协方差（2x2），可直接发布到 PoseWithCovariance。"""
        w = self.weights / max(self.weights.sum(), 1e-12)
        x, y, _ = self.estimate()
        dx = self.particles[:, 0] - x
        dy = self.particles[:, 1] - y
        cov = np.array([[np.sum(w * dx * dx), np.sum(w * dx * dy)],
                        [np.sum(w * dx * dy), np.sum(w * dy * dy)]])
        return cov
