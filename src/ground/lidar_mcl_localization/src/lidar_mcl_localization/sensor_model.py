# -*- coding: utf-8 -*-
"""观测模型：用似然场给每个粒子打分。

为什么用似然场而不是"逐束射线投射"
----------------------------------
朴素做法是对每个粒子、每束激光都做一次完整射线投射，O(N·M·steps)，
几百个粒子就会卡住。似然场把"这条射线打出去会撞到什么"预计算成
一张**到最近障碍物的距离场**，运行时只需把激光末端点变换到地图坐标查表：

    p(z|x) ∝ z_hit · exp(-d² / (2σ²)) + z_rand / z_max

其中 d 是"激光末端点离最近障碍物有多远"。正确的位姿会让大多数末端点落在
障碍物附近（d≈0），权重因此很高。这就是 AMCL 默认的 `likelihood_field` 模型。
"""

import math

import numpy as np


class LikelihoodFieldModel(object):
    """似然场观测模型。

    参数
    ----
    sigma : float
        末端点落点误差的标准差（米）。激光噪声、地图离散化误差都体现在这里。
        典型取值 0.15~0.30；太小会让权重过于尖锐（一束脏数据就打死一个粒子）。
    z_hit, z_rand : float
        命中分量与随机分量的权重，两者之和取 1。
    beam_step : int
        每多少束取一束参与计算。360 束取 8 → 45 束，速度提高 8 倍而精度几乎不变。
    """

    def __init__(self, sigma=0.20, z_hit=0.95, z_rand=0.05, beam_step=8):
        self.sigma = float(sigma)
        self.z_hit = float(z_hit)
        self.z_rand = float(z_rand)
        self.beam_step = max(1, int(beam_step))

    # ------------------------------------------------------------------
    def beam_angles(self, scan):
        """从扫描消息（或字典）里取出每束的角度，返回 (angles, ranges)。"""
        ranges = np.asarray(scan["ranges"], dtype=np.float64)
        n = ranges.size
        if "angle_min" in scan:
            a0 = float(scan["angle_min"])
            inc = float(scan["angle_increment"])
            angles = a0 + inc * np.arange(n)
        else:
            angles = np.linspace(-math.pi, math.pi, n, endpoint=False)
        return angles, ranges

    def subsample(self, angles, ranges, max_range):
        idx = np.arange(0, angles.size, self.beam_step)
        r = ranges[idx]
        a = angles[idx]
        valid = np.isfinite(r) & (r > 0.0) & (r < max_range * 0.999)
        return a[valid], r[valid]

    # ------------------------------------------------------------------
    def log_weights(self, particles, angles, ranges, occ_map, max_range):
        """返回每个粒子的对数权重（未归一化）。

        particles : (N, 3)
        angles, ranges : (M,) 已剔除无效束
        """
        if angles.size == 0:
            return np.zeros(particles.shape[0], dtype=np.float64)

        px = particles[:, 0:1]
        py = particles[:, 1:2]
        pth = particles[:, 2:3]

        world = pth + angles[None, :]                  # (N, M)
        ex = px + ranges[None, :] * np.cos(world)
        ey = py + ranges[None, :] * np.sin(world)

        d = occ_map.likelihood_at_world(ex, ey)        # (N, M) 到最近障碍的距离
        p_hit = np.exp(-0.5 * (d / self.sigma) ** 2)
        p = self.z_hit * p_hit + self.z_rand / max(max_range, 1e-6)
        return np.sum(np.log(np.maximum(p, 1e-12)), axis=1)

    def weights(self, particles, angles, ranges, occ_map, max_range):
        """归一化权重（数值稳定的 exp）。"""
        logw = self.log_weights(particles, angles, ranges, occ_map, max_range)
        if logw.size == 0:
            return np.ones(particles.shape[0]) / particles.shape[0]
        m = float(np.max(logw))
        w = np.exp(logw - m)
        total = float(np.sum(w))
        if not np.isfinite(total) or total <= 0.0:
            return np.ones(particles.shape[0]) / particles.shape[0]
        return w / total


class ScanMatcher(object):
    """粗配准：在上一帧位姿附近做小范围搜索，给出一个"扫描匹配位姿"。

    用途不是替代粒子滤波，而是给 `nn_motion` 提供**监督标签的另一来源**：
    真实机器人没有 /ground_truth，但可以把扫描匹配结果当作"近似真值"。
    这里刻意只做很粗的搜索（步长 5 cm / 2°），够教学演示，也够快。
    """

    def __init__(self, search_xy=0.20, search_th=0.10, xy_step=0.05, th_step=0.035):
        self.search_xy = search_xy
        self.search_th = search_th
        self.xy_step = xy_step
        self.th_step = th_step

    def match(self, occ_map, prior_pose, angles, ranges, max_range):
        best = None
        best_score = -1e18
        offsets = np.arange(-self.search_xy, self.search_xy + 1e-9, self.xy_step)
        for dy in offsets:
            for dx in offsets:
                for dth in np.arange(-self.search_th, self.search_th + 1e-9, self.th_step):
                    cand = (prior_pose[0] + dx, prior_pose[1] + dy, prior_pose[2] + dth)
                    score = self.score(occ_map, cand, angles, ranges, max_range)
                    if score > best_score:
                        best_score = score
                        best = cand
        return best, best_score

    @staticmethod
    def score(occ_map, pose, angles, ranges, max_range):
        x, y, th = pose
        d = occ_map.likelihood_at_world(
            x + ranges * np.cos(th + angles),
            y + ranges * np.sin(th + angles))
        # 命中得分：末端点离障碍越近越好
        return float(-np.sum(np.minimum(d, max_range) ** 2))
