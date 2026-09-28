#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PPO 导航训练环境（纯 numpy 质点 + 速度环 + 随机障碍物）.

无人机简化为带速度环的质点，随机摆放圆柱障碍物与随机目标。观测用激光雷达扇形
直方图（与 CarlaAir 部署节点完全一致），动作为机体系速度，因此训练出的策略
可以直接迁移到真实仿真器。

物理模型：
    yaw += yaw_rate * dt
    pos += R(yaw) · [vx, -vy, vz] * dt      （机体系 前/左/上 -> 世界系）

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
                    build_observation, world_to_body)


class NavEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(
        self,
        bounds: float = 3.0,                    # 场地半边长 (米), x/y ∈ [-bounds, bounds]
        z_range: Tuple[float, float] = (0.3, 3.0),
        goal_radius: float = 0.3,               # 到达判定半径 (米)
        init_z: float = 1.0,
        min_goal_dist: float = 1.5,
        max_goal_dist: float = 3.5,
        num_obstacles: int = 6,
        obstacle_radius: float = 0.35,
        max_steps: int = 400,
        dt: float = 0.1,                        # 控制周期 (秒)
        w_progress: float = 1.0,                # 向目标靠近的进度奖励（每米）
        w_smooth: float = 0.02,                 # 动作平滑惩罚（抑制来回抖动）
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
        self.w_progress = w_progress
        self.w_smooth = w_smooth
        self.w_step = w_step
        self.reward_success = reward_success
        self.reward_crash = reward_crash
        self.rng = np.random.default_rng(seed)

        self.action_space = spaces.Box(-1.0, 1.0, shape=(ACT_DIM,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(OBS_DIM,),
                                            dtype=np.float32)

        # 回合内状态
        self.pos = np.zeros(3)
        self.yaw = 0.0
        self.vel_body = np.zeros(3)              # 当前机体系速度（命令值）
        self.prev_action = np.zeros(ACT_DIM)
        self.goal = np.zeros(3)
        self.start = np.zeros(3)
        self.obstacles_xy = []                   # list[(ox, oy)]
        self.steps = 0
        self.dist_prev = 0.0

    # ------------------------------------------------------------------ 采样
    def _sample_task(self):
        bounds = self.bounds
        r0 = float(self.rng.uniform(1.0, min(2.5, bounds * 0.8)))
        th0 = float(self.rng.uniform(0.0, 2.0 * math.pi))
        self.start = np.array([r0 * math.cos(th0), r0 * math.sin(th0), self.init_z])

        goal = None
        for _ in range(300):
            g = np.array([
                float(self.rng.uniform(-bounds * 0.8, bounds * 0.8)),
                float(self.rng.uniform(-bounds * 0.8, bounds * 0.8)),
                float(self.rng.uniform(0.5, self.z_range[1] - 0.3)),
            ])
            d = float(np.linalg.norm(g - self.start))
            if self.min_goal_dist <= d <= self.max_goal_dist:
                goal = g
                break
        if goal is None:
            goal = np.array([-self.start[0], -self.start[1], self.init_z])
        self.goal = goal

        # 障碍物：随机摆放，避开起点与目标（2D 距离 > 0.9）
        self.obstacles_xy = []
        for _ in range(self.num_obstacles):
            for _ in range(100):
                o = np.array([
                    float(self.rng.uniform(-bounds * 0.7, bounds * 0.7)),
                    float(self.rng.uniform(-bounds * 0.7, bounds * 0.7)),
                ])
                if (np.linalg.norm(o - self.start[:2]) > 0.9
                        and np.linalg.norm(o - self.goal[:2]) > 0.9):
                    self.obstacles_xy.append((o[0], o[1]))
                    break

    # ------------------------------------------------------------------ 传感器仿真
    def _ray_dist(self, dirx: float, diry: float) -> float:
        """从当前位置沿 (dirx,diry) 方向到最近障碍的距离（2D 射线-圆交点）."""
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
        sector = 2.0 * math.pi / N_SECTORS
        hist = np.ones(N_SECTORS, dtype=np.float32)
        for i in range(N_SECTORS):
            a_body = i * sector                        # 扇形中心角（机体系），0 = 正前方
            a_world = a_body + self.yaw
            d = self._ray_dist(math.cos(a_world), math.sin(a_world))
            hist[i] = float(np.clip(d / MAX_RANGE, 0.0, 1.0))
        return hist

    def _obs(self) -> np.ndarray:
        goal_body = world_to_body(self.goal - self.pos, self.yaw)
        return build_observation(self._histogram(), goal_body, self.vel_body)

    # ------------------------------------------------------------------ 回合控制
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self._sample_task()
        self.pos = self.start.copy()
        self.yaw = float(self.rng.uniform(-math.pi, math.pi))
        self.vel_body = np.zeros(3)
        self.prev_action = np.zeros(ACT_DIM)
        self.steps = 0
        self.dist_prev = float(np.linalg.norm(self.pos - self.goal))
        return self._obs(), {}

    def step(self, action):
        a = np.clip(np.asarray(action, dtype=np.float32).reshape(-1)[:ACT_DIM],
                    -1.0, 1.0)
        vx, vy, vz, yaw_rate = action_to_velocity(a)

        # 机体系速度 -> 世界系位移
        self.yaw += yaw_rate * self.dt
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.pos += self.dt * np.array([vx * c - vy * s,
                                        vx * s + vy * c,
                                        vz])
        self.vel_body = np.array([vx, vy, vz], dtype=np.float64)

        dist = float(np.linalg.norm(self.pos - self.goal))

        # ---- 终止判定 ----
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

        # ---- 奖励 ----
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
