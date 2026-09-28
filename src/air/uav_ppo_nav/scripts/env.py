#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PPO 导航训练环境（纯 numpy 质点 + 世界系速度 + 随机障碍物）.

**坐标系**：全部世界系（ENU）。原因是 CarlaAir 里 SimpleFlight 的偏航保持不生效，
无人机机头会自行漂移；若用机体系控制，轨迹会随偏航打转。改用世界系后，
机头怎么转都不影响轨迹，策略才能稳定收敛。

物理模型（含速度环一阶滞后，贴近真实飞行器）：
    vel += (cmd_vel - vel) * dt / vel_tau
    pos += vel * dt
任务：从随机起点飞到随机目标点，途中避开圆柱障碍物。
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from policy import (ACT_DIM, MAX_HORIZ_SPEED, MAX_RANGE, MAX_VERT_SPEED,
                    N_SECTORS, OBS_DIM, action_to_velocity,
                    build_observation, lidar_points_to_histogram)


class NavEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(
        self,
        bounds: float = 10.0,                   # 场地半边长 (米)，接近 CarlaAir 城镇尺度
        z_range: Tuple[float, float] = (-4.0, 4.0),
        goal_radius: float = 0.4,               # 到达判定半径 (米)
        init_z: float = 1.5,
        min_goal_dist: float = 2.0,
        max_goal_dist: float = 8.0,
        num_obstacles: int = 16,
        obstacle_radius: float = 0.6,
        max_steps: int = 400,
        dt: float = 0.1,                        # 控制周期 (秒)
        vel_tau: float = 0.45,                  # 速度环一阶响应时间常数 (秒)
        w_progress: float = 1.0,                # 向目标靠近的进度奖励（每米）
        w_smooth: float = 0.02,                 # 动作平滑惩罚
        w_step: float = 0.005,                  # 每步时间惩罚
        reward_success: float = 20.0,
        reward_crash: float = -20.0,
        seed: Optional[int] = None,
    ):
        super().__init__()
        self.bounds = bounds
        self.z_range = z_range
        self.goal_radius = goal_radius
        self.init_z = init_z
        self.min_goal_dist = min_goal_dist
        self.max_goal_dist = max_goal_dist
        self.num_obstacles = num_obstacles
        self.obstacle_radius = obstacle_radius
        self.max_steps = max_steps
        self.dt = dt
        self.vel_tau = vel_tau
        self.w_progress = w_progress
        self.w_smooth = w_smooth
        self.w_step = w_step
        self.reward_success = reward_success
        self.reward_crash = reward_crash
        self.rng = np.random.default_rng(seed)

        self.action_space = spaces.Box(-1.0, 1.0, shape=(ACT_DIM,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(OBS_DIM,),
                                            dtype=np.float32)

        self.pos = np.zeros(3)          # 世界系 ENU
        self.vel = np.zeros(3)          # 世界系 ENU（实际速度）
        self.yaw = 0.0                  # 不影响运动，仅用于保持接口一致
        self.prev_action = np.zeros(ACT_DIM)
        self.goal = np.zeros(3)
        self.start = np.zeros(3)
        self.obstacles_xy = []
        self.steps = 0
        self.dist_prev = 0.0

    # ------------------------------------------------------------------ 采样
    def _sample_task(self):
        bounds = self.bounds
        r0 = float(self.rng.uniform(1.5, min(5.0, bounds * 0.8)))
        th0 = float(self.rng.uniform(0.0, 2.0 * math.pi))
        self.start = np.array([r0 * math.cos(th0), r0 * math.sin(th0), self.init_z])

        goal = None
        for _ in range(300):
            g = np.array([
                float(self.rng.uniform(-bounds * 0.8, bounds * 0.8)),
                float(self.rng.uniform(-bounds * 0.8, bounds * 0.8)),
                float(self.rng.uniform(self.z_range[0] * 0.5, self.z_range[1] * 0.5)),
            ])
            d = float(np.linalg.norm(g - self.start))
            if self.min_goal_dist <= d <= self.max_goal_dist:
                goal = g
                break
        if goal is None:
            goal = np.array([-self.start[0], -self.start[1], self.init_z])
        self.goal = goal

        self.obstacles_xy = []
        for _ in range(self.num_obstacles):
            for _ in range(100):
                o = np.array([
                    float(self.rng.uniform(-bounds * 0.7, bounds * 0.7)),
                    float(self.rng.uniform(-bounds * 0.7, bounds * 0.7)),
                ])
                if (np.linalg.norm(o - self.start[:2]) > 1.2
                        and np.linalg.norm(o - self.goal[:2]) > 1.2):
                    self.obstacles_xy.append((o[0], o[1]))
                    break

    # ------------------------------------------------------------------ 传感器仿真
    def _ray_dist(self, dirx: float, diry: float) -> float:
        """从当前位置沿世界系方向 (dirx,diry) 到最近障碍的距离（2D 射线-圆交点）."""
        d = MAX_RANGE
        px, py = self.pos[0], self.pos[1]
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
        """世界系固定方向的扇形直方图（用于与真实激光雷达对照）."""
        sector = 2.0 * math.pi / N_SECTORS
        hist = np.ones(N_SECTORS, dtype=np.float32)
        for i in range(N_SECTORS):
            a = i * sector                      # 世界系角度，0 = 正东
            d = self._ray_dist(math.cos(a), math.sin(a))
            hist[i] = float(np.clip(d / MAX_RANGE, 0.0, 1.0))
        return hist

    def _obs(self) -> np.ndarray:
        return build_observation(self._histogram(), self.goal - self.pos, self.vel)

    # ------------------------------------------------------------------ 回合控制
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self._sample_task()
        self.pos = self.start.copy()
        self.vel = np.zeros(3)
        self.yaw = float(self.rng.uniform(-math.pi, math.pi))
        self.prev_action = np.zeros(ACT_DIM)
        self.steps = 0
        self.dist_prev = float(np.linalg.norm(self.pos - self.goal))
        return self._obs(), {}

    def step(self, action):
        a = np.clip(np.asarray(action, dtype=np.float32).reshape(-1)[:ACT_DIM],
                    -1.0, 1.0)
        vx, vy, vz, yaw_rate = action_to_velocity(a)

        # 一阶速度响应：真实飞行器的速度环有滞后，指令速度不是瞬时达到的
        cmd = np.array([vx, vy, vz], dtype=np.float64)
        alpha = min(1.0, self.dt / max(self.vel_tau, 1e-3))
        self.vel += alpha * (cmd - self.vel)

        self.pos += self.dt * self.vel
        self.yaw += yaw_rate * self.dt           # 不影响运动

        dist = float(np.linalg.norm(self.pos - self.goal))

        crash_reason = None
        if dist < self.goal_radius:
            terminated, success = True, True
        else:
            terminated, success = False, False
            if not (self.z_range[0] <= self.pos[2] <= self.z_range[1]):
                crash_reason = "超出高度范围"
            elif abs(self.pos[0]) > self.bounds + 0.5 or abs(self.pos[1]) > self.bounds + 0.5:
                crash_reason = "飞出场地"
            else:
                for (ox, oy) in self.obstacles_xy:
                    if (self.pos[0] - ox) ** 2 + (self.pos[1] - oy) ** 2 < self.obstacle_radius ** 2:
                        crash_reason = "撞障碍"
                        break
            if crash_reason is not None:
                terminated = True

        truncated = (not terminated) and (self.steps >= self.max_steps - 1)

        progress = self.dist_prev - dist
        smooth = float(np.sum(np.square(a - self.prev_action)))
        reward = (self.w_progress * progress
                  - self.w_smooth * smooth
                  - self.w_step)
        if success:
            reward += self.reward_success
        elif terminated:
            reward += self.reward_crash

        self.dist_prev = dist
        self.prev_action = a
        self.steps += 1

        info = {"success": success, "distance": dist, "crash_reason": crash_reason}
        return self._obs(), float(reward), terminated, truncated, info

    def close(self):
        pass
