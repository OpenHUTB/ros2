#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""城市动态交通避障训练环境（纯 numpy 质点 + 世界系速度 + 移动车辆）.

与 ``uav_ppo_nav`` 的区别：障碍不是静止圆柱，而是**沿车道行驶的车辆**。
车辆位置、速度随时间变化，策略必须结合"相对速度"预判才能安全穿过车流。

物理模型（与 uav_ppo_nav 一致，含速度环一阶滞后）：
    vel += (cmd_vel - vel) * dt / vel_tau
    pos += vel * dt

高度：本模块聚焦**水平面**动态避障，无人机固定在 ``init_z``；
动作里的 ``vz`` 保留以复用现有接口，但底层执行器应保持高度
（三维避障留给 ``uav_ppo_3d_nav`` 方向）。
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from policy import (ACT_DIM, MAX_HORIZ_SPEED, MAX_RANGE, MAX_VERT_SPEED,
                    N_SECTORS, build_observation, observation_dim,
                    traffic_features)


class TrafficNavEnv(gym.Env):
    """固定高度、随机起终点、三条车道的动态交通场景."""

    metadata = {"render_modes": []}

    def __init__(
        self,
        bounds: float = 10.0,
        lane_ys: Tuple[float, ...] = (-4.0, 4.0),
        init_z: float = 1.2,
        goal_radius: float = 0.6,
        drone_radius: float = 0.3,
        car_radius: float = 1.0,
        cars_per_lane: int = 2,
        car_speed: Tuple[float, float] = (3.0, 6.0),
        static_obstacles: int = 2,
        obstacle_radius: float = 0.6,
        min_goal_dist: float = 6.0,
        max_goal_dist: float = 16.0,
        max_steps: int = 500,
        dt: float = 0.1,
        vel_tau: float = 0.45,
        w_progress: float = 1.0,
        w_smooth: float = 0.02,
        w_step: float = 0.005,
        w_ttc: float = 1.0,
        w_proximity: float = 0.0,
        proximity_range: float = 2.0,
        ttc_horizon: float = 2.0,
        reward_success: float = 20.0,
        reward_crash: float = -30.0,
        include_traffic: bool = True,
        force_crossing: bool = False,
        seed: Optional[int] = None,
    ):
        super().__init__()
        self.bounds = float(bounds)
        self.lane_ys = tuple(float(y) for y in lane_ys)
        self.init_z = float(init_z)
        self.goal_radius = float(goal_radius)
        self.drone_radius = float(drone_radius)
        self.car_radius = float(car_radius)
        self.base_cars_per_lane = int(cars_per_lane)
        self.max_cars_per_lane = max(2, int(cars_per_lane))
        self.car_speed_range = (float(car_speed[0]), float(car_speed[1]))
        self.static_obstacles = int(static_obstacles)
        self.obstacle_radius = float(obstacle_radius)
        self.min_goal_dist = float(min_goal_dist)
        self.max_goal_dist = float(max_goal_dist)
        self.max_steps = int(max_steps)
        self.dt = float(dt)
        self.vel_tau = float(vel_tau)
        self.w_progress = float(w_progress)
        self.w_smooth = float(w_smooth)
        self.w_step = float(w_step)
        self.w_ttc = float(w_ttc)
        self.w_proximity = float(w_proximity)
        self.proximity_range = float(proximity_range)
        self.ttc_horizon = float(ttc_horizon)
        self.reward_success = float(reward_success)
        self.reward_crash = float(reward_crash)
        self.include_traffic = bool(include_traffic)
        self.force_crossing = bool(force_crossing)
        self.rng = np.random.default_rng(seed)

        self.difficulty = 0.0      # 0..1，课程学习由外部回调设置
        self.action_space = spaces.Box(-1.0, 1.0, shape=(ACT_DIM,), dtype=np.float32)
        dim = observation_dim(self.include_traffic)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(dim,),
                                            dtype=np.float32)

        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.yaw = 0.0
        self.prev_action = np.zeros(ACT_DIM)
        self.goal = np.zeros(3)
        self.start = np.zeros(3)
        self.cars: List[Dict[str, float]] = []
        self.obstacles_xy: List[Tuple[float, float]] = []
        self.steps = 0
        self.dist_prev = 0.0

    # ------------------------------------------------------------------ 难度
    def set_difficulty(self, value: float) -> None:
        """课程学习：0 = 每车道 1 辆慢车，1 = 每车道 3 辆快车."""
        self.difficulty = float(np.clip(value, 0.0, 1.0))

    def _cars_per_lane(self) -> int:
        span = self.max_cars_per_lane - self.base_cars_per_lane
        return int(round(self.base_cars_per_lane + self.difficulty * span))

    def _speed_scale(self) -> float:
        return 0.6 + 0.4 * self.difficulty

    # ------------------------------------------------------------------ 场景采样
    def _clearance_to_cars(self, x: float, y: float) -> float:
        if not self.cars:
            return float("inf")
        return min(math.hypot(x - c["x"], y - c["y"]) for c in self.cars)

    def _sample_cars(self) -> None:
        bounds = self.bounds
        self.cars = []
        per_lane = self._cars_per_lane()
        for y in self.lane_ys:
            for _ in range(per_lane):
                speed = float(self.rng.uniform(*self.car_speed_range)) * self._speed_scale()
                direction = 1.0 if self.rng.random() < 0.5 else -1.0
                self.cars.append({
                    "x": float(self.rng.uniform(-bounds, bounds)),
                    "y": y,
                    "vx": direction * speed,
                    "vy": 0.0,
                })

    def _sample_task(self) -> None:
        bounds = self.bounds
        # 先放车，再挑一个**不与车重叠**的出生点：否则回合一开始就判定碰撞，
        # 训练信号会被"出生即撞"污染（实测约 14% 的重置会落在车体内）。
        self._sample_cars()
        max_lane = max(abs(y) for y in self.lane_ys)
        if self.force_crossing:
            # 起点固定在路一侧、终点固定在另一侧：任何策略都必须横穿全部车道，
            # 这样"直接朝目标飞"的贪心策略才会稳定吃亏。
            side = -1.0 if self.rng.random() < 0.5 else 1.0
            y0_range = (max_lane + 2.5, bounds * 0.8)
            y1_range = (-bounds * 0.8, -(max_lane + 2.5))
            if side < 0:
                y0_range, y1_range = (-bounds * 0.8, -(max_lane + 2.5)), (max_lane + 2.5,
                                                                           bounds * 0.8)
            best = None
            for _ in range(200):
                x = float(self.rng.uniform(-bounds * 0.7, bounds * 0.7))
                y = float(self.rng.uniform(*y0_range))
                clearance = self._clearance_to_cars(x, y)
                if best is None or clearance > best[0]:
                    best = (clearance, x, y)
                if clearance > 2.5:
                    break
            _, sx, sy = best
            self.start = np.array([sx, sy, self.init_z])

            goal = None
            for _ in range(400):
                gx = float(self.rng.uniform(-bounds * 0.7, bounds * 0.7))
                gy = float(self.rng.uniform(*y1_range))
                g = np.array([gx, gy, self.init_z])
                d = float(np.linalg.norm(g - self.start))
                if (self.min_goal_dist <= d <= self.max_goal_dist
                        and self._clearance_to_cars(gx, gy) > 2.5):
                    goal = g
                    break
            self.goal = goal if goal is not None else np.array([-self.start[0], -self.start[1],
                                                                self.init_z])
        else:
            best = None
            for _ in range(200):
                x = float(self.rng.uniform(-bounds * 0.7, bounds * 0.7))
                y = float(self.rng.uniform(-bounds * 0.7, bounds * 0.7))
                clearance = self._clearance_to_cars(x, y)
                if best is None or clearance > best[0]:
                    best = (clearance, x, y)
                if clearance > 2.5:
                    break
            _, sx, sy = best
            self.start = np.array([sx, sy, self.init_z])

            goal = None
            for _ in range(400):
                g = np.array([self.rng.uniform(-bounds * 0.8, bounds * 0.8),
                              self.rng.uniform(-bounds * 0.8, bounds * 0.8),
                              self.init_z])
                d = float(np.linalg.norm(g - self.start))
                if (self.min_goal_dist <= d <= self.max_goal_dist
                        and self._clearance_to_cars(g[0], g[1]) > 2.5):
                    goal = g
                    break
            self.goal = goal if goal is not None else np.array([-self.start[0], -self.start[1],
                                                                self.init_z])

        self.obstacles_xy = []
        for _ in range(self.static_obstacles):
            for _ in range(100):
                o = (float(self.rng.uniform(-bounds * 0.7, bounds * 0.7)),
                     float(self.rng.uniform(-bounds * 0.7, bounds * 0.7)))
                if (math.hypot(o[0] - self.start[0], o[1] - self.start[1]) > 2.0
                        and math.hypot(o[0] - self.goal[0], o[1] - self.goal[1]) > 2.0
                        and all(abs(o[1] - y) > 1.6 for y in self.lane_ys)):
                    self.obstacles_xy.append(o)
                    break

    # ------------------------------------------------------------------ 传感器
    def _ray_dist(self, dirx: float, diry: float) -> float:
        """从当前位置沿世界系方向到最近障碍（圆）的距离."""
        d = MAX_RANGE
        px, py = self.pos[0], self.pos[1]
        for (ox, oy) in self.obstacles_xy:
            d = min(d, self._circle_hit(px, py, dirx, diry, ox, oy, self.obstacle_radius))
        for car in self.cars:
            d = min(d, self._circle_hit(px, py, dirx, diry, car["x"], car["y"], self.car_radius))
        return d

    @staticmethod
    def _circle_hit(px, py, dirx, diry, ox, oy, radius) -> float:
        fx, fy = px - ox, py - oy
        b = 2.0 * (fx * dirx + fy * diry)
        c = fx * fx + fy * fy - radius * radius
        disc = b * b - 4.0 * c
        if disc < 0.0:
            return MAX_RANGE
        sq = math.sqrt(disc)
        t = (-b - sq) / 2.0
        if t < 0.0:
            t = (-b + sq) / 2.0
        return t if t >= 0.0 else MAX_RANGE

    def _histogram(self) -> np.ndarray:
        sector = 2.0 * math.pi / N_SECTORS
        hist = np.ones(N_SECTORS, dtype=np.float32)
        for i in range(N_SECTORS):
            a = i * sector
            d = self._ray_dist(math.cos(a), math.sin(a))
            hist[i] = float(np.clip(d / MAX_RANGE, 0.0, 1.0))
        return hist

    def _obs(self) -> np.ndarray:
        traffic = traffic_features(
            [(c["x"], c["y"], c["vx"], c["vy"]) for c in self.cars],
            drone_xy=(self.pos[0], self.pos[1]),
            drone_vel=(self.vel[0], self.vel[1]))
        return build_observation(self._histogram(), self.goal - self.pos, self.vel,
                                 traffic=traffic, include_traffic=self.include_traffic)

    # ------------------------------------------------------------------ 碰撞/风险
    def _min_clearance(self) -> float:
        """到最近车辆的**表面**距离（米），可为负（已重叠）."""
        if not self.cars:
            return float("inf")
        return min(math.hypot(self.pos[0] - c["x"], self.pos[1] - c["y"])
                   - (self.drone_radius + self.car_radius) for c in self.cars)

    def _ttc_values(self) -> List[float]:
        """当前与每辆车的碰撞时间（只保留在 ttc_horizon 内的）."""
        from policy import time_to_collision
        out = []
        for car in self.cars:
            ttc = time_to_collision((self.pos[0], self.pos[1]),
                                    (self.vel[0], self.vel[1]),
                                    (car["x"], car["y"]), (car["vx"], car["vy"]),
                                    max_time=self.ttc_horizon)
            if ttc is not None:
                out.append(ttc)
        return out

    def _advance_cars(self) -> None:
        limit = self.bounds * 1.15
        for car in self.cars:
            car["x"] += car["vx"] * self.dt
            car["y"] += car["vy"] * self.dt
            if abs(car["x"]) > limit:
                car["x"] = -math.copysign(limit, car["vx"])
                car["vx"] = float(self.rng.uniform(*self.car_speed_range)) * self._speed_scale() \
                    * (1.0 if car["vx"] > 0 else -1.0)

    # ------------------------------------------------------------------ gym 接口
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
        return self._obs(), {"difficulty": self.difficulty,
                             "cars": len(self.cars)}

    def step(self, action):
        a = np.clip(np.asarray(action, dtype=np.float32).reshape(-1)[:ACT_DIM], -1.0, 1.0)
        cmd = np.array([a[0] * MAX_HORIZ_SPEED,
                        a[1] * MAX_HORIZ_SPEED,
                        0.0])                      # 高度由执行器保持
        alpha = min(1.0, self.dt / max(self.vel_tau, 1e-3))
        self.vel += alpha * (cmd - self.vel)

        ttc_before = self._ttc_values()
        self.pos += self.dt * self.vel
        self.pos[2] = self.init_z
        self.yaw += float(a[3] * 1.0) * self.dt   # 不影响运动
        self._advance_cars()

        dist = float(np.linalg.norm(self.pos - self.goal))
        clearance = self._min_clearance()
        terminated, success, crash_reason = False, False, None
        if dist < self.goal_radius:
            terminated, success = True, True
        elif clearance <= 0.0:
            terminated, crash_reason = True, "撞车辆"
        elif abs(self.pos[0]) > self.bounds + 0.5 or abs(self.pos[1]) > self.bounds + 0.5:
            terminated, crash_reason = True, "飞出场地"
        else:
            for (ox, oy) in self.obstacles_xy:
                if math.hypot(self.pos[0] - ox, self.pos[1] - oy) < self.obstacle_radius + self.drone_radius:
                    terminated, crash_reason = True, "撞静态障碍"
                    break

        truncated = (not terminated) and (self.steps >= self.max_steps - 1)

        progress = self.dist_prev - dist
        reward = (self.w_progress * progress
                  - self.w_smooth * float(np.sum(np.square(a - self.prev_action)))
                  - self.w_step)
        if clearance < self.proximity_range:
            reward -= self.w_proximity * (self.proximity_range - clearance)
        if ttc_before:
            reward -= self.w_ttc * float(np.sum(
                np.square(1.0 - np.asarray(ttc_before) / self.ttc_horizon)))
        if success:
            reward += self.reward_success
        elif terminated:
            reward += self.reward_crash

        self.dist_prev = dist
        self.prev_action = a
        self.steps += 1

        info = {"success": success, "distance": dist, "crash_reason": crash_reason,
                "min_clearance": clearance,
                "min_ttc": min(ttc_before) if ttc_before else float("nan")}
        return self._obs(), float(reward), terminated, truncated, info

    def close(self):
        pass
