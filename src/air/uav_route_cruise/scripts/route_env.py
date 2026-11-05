#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""固定路线巡航环境（纯 numpy 质点 + 世界系速度 + 随机障碍）.

任务
----
无人机从航线起点出发，沿**固定闭合航线**巡航一整圈回到起点。
障碍物每个回合随机摆放（但不会压在航线上），所以策略必须学会「沿航线 + 避障」两件事。

与 uav_ppo_nav.env 的关系
-------------------------
物理模型（速度一阶滞后、世界系 ENU 控制）**完全沿用**同一套设定；
差别只在任务：那边是"随机目标点"，这边是"沿固定航线循迹"，
因此观测里多了航线相关量（横向偏差 / 航向偏差 / 前方航点 / 进度），奖励主项也换成
**沿航线的前进量**而不是"到目标的距离减少量"。

观测（30 维，全部世界系；维数与 OBS_DIM 常量一致）
-----------------------------------------------
    [0:24]   24 个扇形激光距离 / MAX_RANGE（1 = 无遮挡）
    [24]     横向偏差 / CROSS_SCALE      （正 = 偏航线左侧）
    [25]     航向偏差 / π                （速度方向与航线切向的夹角）
    [26]     前视点相对位置 x / LOOKAHEAD_SCALE（4 m 处）
    [27]     前视点相对位置 y / LOOKAHEAD_SCALE
    [28]     当前速度水平分量 / MAX_HORIZ_SPEED
    [29]     高度偏差 / Z_SCALE          （相对航线高度）

动作（4 维）
------------
    [vx, vy, vz, yaw_rate] ∈ [-1,1]^4 -> 世界系速度（ENU）+ 偏航角速度
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import math
from typing import Optional, Tuple

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from route import Route, make_route

# ---- 与 uav_ppo_nav.policy 保持一致的量纲 ----
N_SECTORS = 24
MAX_RANGE = 12.0
MAX_HORIZ_SPEED = 3.0
MAX_VERT_SPEED = 1.5
MAX_YAW_RATE = 1.0
ACT_DIM = 4

# ---- 本模块新增 ----
CROSS_SCALE = 4.0          # 横向偏差归一化尺度 (m)
LOOKAHEAD_SCALE = 6.0      # 前方航点相对位置归一化尺度 (m)
Z_SCALE = 2.0              # 高度偏差归一化尺度 (m)
OBS_DIM = N_SECTORS + 6    # 24 激光 + 6 航线相关 = 30


class RouteEnv(gym.Env):
    """沿固定闭合航线巡航的环境。"""

    metadata = {"render_modes": []}

    def __init__(
        self,
        route_name: str = "rect",
        route: Optional[Route] = None,
        num_obstacles: int = 14,
        obstacle_radius: float = 0.6,
        obstacle_clearance: float = 1.1,     # 障碍与航线的最小距离，避免堵死航线
        on_route_frac: float = 0.25,         # 直接压在航线上的障碍比例：必须绕行
        z_range: Tuple[float, float] = (-4.0, 4.0),
        max_steps: int = 600,
        dt: float = 0.1,
        vel_tau: float = 0.45,
        lookahead: float = 4.0,
        start_jitter: float = 0.4,
        w_progress: float = 10.0,            # 沿航线前进 1 m 的奖励
        w_cross: float = 0.2,                # 横向偏差平方惩罚（调小以便允许绕行）
        w_smooth: float = 0.02,              # 动作平滑惩罚
        w_step: float = 0.005,               # 每步时间惩罚
        w_height: float = 0.5,               # 高度偏差惩罚
        w_prox: float = 0.0,                 # 靠近障碍的连续惩罚（实测调大反而更差，默认关闭）
        prox_range: float = 1.2,             # 触发近距惩罚的净距离 (m)
        reward_lap: float = 120.0,           # 完成一圈的奖励
        reward_crash: float = -200.0,
        seed: Optional[int] = None,
    ):
        super().__init__()
        self.route = route or make_route(route_name)
        self.num_obstacles = int(num_obstacles)
        self.obstacle_radius = float(obstacle_radius)
        self.obstacle_clearance = float(obstacle_clearance)
        self.on_route_frac = float(on_route_frac)
        self.z_range = z_range
        self.max_steps = int(max_steps)
        self.dt = float(dt)
        self.vel_tau = float(vel_tau)
        self.lookahead = float(lookahead)
        self.start_jitter = float(start_jitter)
        self.w_progress = float(w_progress)
        self.w_cross = float(w_cross)
        self.w_smooth = float(w_smooth)
        self.w_step = float(w_step)
        self.w_height = float(w_height)
        self.w_prox = float(w_prox)
        self.prox_range = float(prox_range)
        self.reward_lap = float(reward_lap)
        self.reward_crash = float(reward_crash)
        self.rng = np.random.default_rng(seed)

        self.action_space = spaces.Box(-1.0, 1.0, shape=(ACT_DIM,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(OBS_DIM,),
                                            dtype=np.float32)

        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.yaw = 0.0
        self.prev_action = np.zeros(ACT_DIM)
        self.obstacles_xy = []
        self.steps = 0
        self.visited_s = []
        self.lap_done = False

    # ------------------------------------------------------------ 场景
    def _sample_obstacles(self):
        """随机摆障碍。

        其中 `on_route_frac` 比例的障碍**直接压在航线上**（离航线中心小于半径），
        因此「贴着航线中线飞」必然撞上 —— 想通过就必须偏离航线绕行。
        这正是本任务需要学习的部分：光会循迹不够，还要会临时绕避。
        剩下那部分放在航线旁侧，保留 obstacle_clearance 的通行余量。
        """
        self.obstacles_xy = []
        n_on = int(round(self.num_obstacles * max(0.0, min(1.0, self.on_route_frac))))
        tries = 0
        while len(self.obstacles_xy) < self.num_obstacles and tries < 8000:
            tries += 1
            o = self.rng.uniform(-1.0, 1.0, 2) * np.array([11.0, 9.0])
            _, _, _, signed = self.route.project(o)
            if len(self.obstacles_xy) < n_on:
                # 压在航线上：必须绕行
                if abs(signed) > self.obstacle_radius * 0.6:
                    continue
            elif abs(signed) < (self.obstacle_clearance + self.obstacle_radius):
                continue
            if any((o[0] - p[0]) ** 2 + (o[1] - p[1]) ** 2 < (2.0 * self.obstacle_radius) ** 2
                   for p in self.obstacles_xy):
                continue
            self.obstacles_xy.append((float(o[0]), float(o[1])))

    # ------------------------------------------------------------ 传感器
    def _ray_dist(self, dirx: float, diry: float) -> float:
        """世界系方向上的最近障碍距离（2D 射线-圆交点）。"""
        d = MAX_RANGE
        px, py = float(self.pos[0]), float(self.pos[1])
        for (ox, oy) in self.obstacles_xy:
            fx, fy = px - ox, py - oy
            b = 2.0 * (fx * dirx + fy * diry)
            c = fx * fx + fy * fy - self.obstacle_radius ** 2
            disc = b * b - 4.0 * c
            if disc < 0.0:
                continue
            sq = math.sqrt(disc)
            t = (-b - sq) / 2.0
            if t < 0.0:
                t = (-b + sq) / 2.0
            if t < 0.0:
                continue
            d = min(d, t)
        return d

    def _histogram(self) -> np.ndarray:
        sector = 2.0 * math.pi / N_SECTORS
        hist = np.ones(N_SECTORS, dtype=np.float32)
        for i in range(N_SECTORS):
            a = i * sector
            hist[i] = float(np.clip(self._ray_dist(math.cos(a), math.sin(a)) / MAX_RANGE,
                                    0.0, 1.0))
        return hist

    def _obs(self) -> np.ndarray:
        i, s, proj, cross = self.route.project(self.pos[:2])
        # 航线切向（世界系）
        d = self.route.seg[i, :2]
        n = np.linalg.norm(d) or 1.0
        tangent = d / n
        # 航向偏差：当前速度方向与切向的夹角（速度很小就用机体朝向）
        v = self.vel[:2]
        vn = np.linalg.norm(v)
        heading = v / vn if vn > 1e-3 else np.array([math.cos(self.yaw), math.sin(self.yaw)])
        dot = float(np.clip(heading @ tangent, -1.0, 1.0))
        crossang = float(np.arccos(dot))
        # 左侧/右侧符号（与横向偏差一致）
        if (tangent[0] * heading[1] - tangent[1] * heading[0]) < 0:
            crossang = -crossang
        # 单个前视点（4 m）。试过给 3 个点（4/8/12 m）让它预判弯道，
        # 实测观测维度变多之后训练反而更难、成功率从 77% 掉到 63%，故回退。
        tgt = self.route.waypoint_ahead(self.pos[:2], s, self.lookahead)
        rel = tgt[:2] - self.pos[:2]
        z_err = float(self.pos[2] - self.route.points[0, 2])
        extra = np.array([
            float(np.clip(cross / CROSS_SCALE, -3.0, 3.0)),
            float(crossang / math.pi),
            float(np.clip(rel[0] / LOOKAHEAD_SCALE, -3.0, 3.0)),
            float(np.clip(rel[1] / LOOKAHEAD_SCALE, -3.0, 3.0)),
            float(vn / MAX_HORIZ_SPEED),
            float(np.clip(z_err / Z_SCALE, -3.0, 3.0)),
        ], dtype=np.float32)
        return np.concatenate([self._histogram(), extra]).astype(np.float32)

    # ------------------------------------------------------------ 回合
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self._sample_obstacles()
        # 起点：航线起点 + 小幅抖动
        p0 = self.route.points[0, :2] + self.rng.uniform(-self.start_jitter,
                                                         self.start_jitter, 2)
        self.pos = np.array([p0[0], p0[1], self.route.points[0, 2]], dtype=np.float64)
        self.vel = np.zeros(3)
        d = self.route.seg[0, :2]
        self.yaw = float(math.atan2(d[1], d[0]))
        self.prev_action = np.zeros(ACT_DIM)
        self.steps = 0
        self.lap_done = False
        _, s, _, _ = self.route.project(self.pos[:2])
        self.visited_s = [s]
        self._s_prev = s
        self._last_s = s
        return self._obs(), {}

    def step(self, action):
        a = np.clip(np.asarray(action, dtype=np.float64).reshape(-1)[:ACT_DIM],
                    -1.0, 1.0)
        vx = float(a[0] * MAX_HORIZ_SPEED)
        vy = float(a[1] * MAX_HORIZ_SPEED)
        vz = float(a[2] * MAX_VERT_SPEED)
        yaw_rate = float(a[3] * MAX_YAW_RATE)

        # 一阶速度响应（与 uav_ppo_nav 一致）
        cmd = np.array([vx, vy, vz], dtype=np.float64)
        alpha = min(1.0, self.dt / max(self.vel_tau, 1e-3))
        self.vel += alpha * (cmd - self.vel)
        self.pos += self.dt * self.vel
        self.yaw += yaw_rate * self.dt

        # 航线投影与前进量（处理跨越起终点时的环绕）
        i, s, proj, cross = self.route.project(self.pos[:2])
        ds = s - self._s_prev
        if ds < -self.route.length / 2.0:
            ds += self.route.length          # 正向跨过起点
        elif ds > self.route.length / 2.0:
            ds -= self.route.length          # 倒着跨过起点
        self._s_prev = s
        self.visited_s.append(s)

        # 完成一圈：累计前进量达到整条航线长度
        if not self.lap_done:
            total = 0.0
            prev = self.visited_s[0]
            for sv in self.visited_s[1:]:
                d = sv - prev
                if d < -self.route.length / 2.0:
                    d += self.route.length
                elif d > self.route.length / 2.0:
                    d -= self.route.length
                total += d
                prev = sv
            if total >= self.route.length:
                self.lap_done = True

        # 碰撞 / 越界判定
        crash_reason = None
        if not (self.z_range[0] <= self.pos[2] <= self.z_range[1]):
            crash_reason = "超出高度范围"
        elif abs(self.pos[0]) > 12.0 or abs(self.pos[1]) > 10.0:
            crash_reason = "飞出场地"
        else:
            for (ox, oy) in self.obstacles_xy:
                if (self.pos[0] - ox) ** 2 + (self.pos[1] - oy) ** 2 < self.obstacle_radius ** 2:
                    crash_reason = "撞障碍"
                    break

        terminated = bool(crash_reason is not None or self.lap_done)
        truncated = (not terminated) and (self.steps >= self.max_steps - 1)

        smooth = float(np.sum(np.square(a - self.prev_action)))
        z_err = float(self.pos[2] - self.route.points[0, 2])
        # 近距惩罚：离障碍越近扣得越多。
        # 只靠「撞上才扣 200」是稀疏信号，策略很难学到提前规避；
        # 加一个连续项后，它才能从「开始靠近」就感受到代价。
        clearance = float("inf")
        for (ox, oy) in self.obstacles_xy:
            d = math.hypot(self.pos[0] - ox, self.pos[1] - oy) - self.obstacle_radius
            clearance = min(clearance, d)
        prox = max(0.0, self.prox_range - clearance) if np.isfinite(clearance) else 0.0
        reward = (self.w_progress * ds
                  - self.w_cross * (cross ** 2)
                  - self.w_height * (z_err ** 2)
                  - self.w_smooth * smooth
                  - self.w_step
                  - self.w_prox * (prox ** 2))
        if self.lap_done:
            reward += self.reward_lap
        elif crash_reason is not None:
            reward += self.reward_crash

        self.prev_action = a
        self.steps += 1
        info = {"success": self.lap_done, "crash_reason": crash_reason,
                "cross": float(cross), "s": float(s),
                "progress": float(min(1.0, s / self.route.length))}
        return self._obs(), float(reward), terminated, truncated, info

    def close(self):
        pass
