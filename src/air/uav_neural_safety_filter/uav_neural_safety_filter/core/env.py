#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""障碍场安全飞行环境：动作语义为机头系速度，观测为 3×36 距离栅格。

标称控制器只朝目标飞，不避障；安全过滤器在这一层之上修正指令。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from uav_mujoco_sim.core import GeometricController, Quadrotor
from uav_mujoco_sim.core.scenes import (Scene, SceneObject, make_scene,
                                        obstacle_field)
from uav_mujoco_sim.core.sensors import LidarSensor

from .geometry import clearance_at, rollout_clearance_many, swept_clearance
from .ranges import (FEAT_DIM, MAX_RANGE, RangeConfig, build_features,
                     grid_min_range, scan_grid)

__all__ = ["SIM_DT", "CTRL_HZ", "FRAME_DT", "EnvConfig", "EpisodeSpec",
           "make_episodes", "SafetyEnv"]

SIM_DT = 0.01
CTRL_HZ = 20.0
FRAME_DT = 1.0 / CTRL_HZ
GOAL_RADIUS = 0.8


@dataclass
class EpisodeSpec:
    seed: int
    start: np.ndarray
    goal: np.ndarray
    label: str = ""

    def key(self) -> str:
        return self.label or "seed%d" % self.seed


def safety_field(seed: int = 0, n_obstacles: int = 20,
                 area: float = 9.0) -> Scene:
    """安全评测场景：高度 ≥1.2 m 的柱状障碍，三层激光都能打到。"""
    rng = np.random.default_rng(seed)
    objs: List[SceneObject] = []
    for i in range(n_obstacles):
        while True:
            x = float(rng.uniform(-area, area))
            y = float(rng.uniform(-area, area))
            if math.hypot(x, y) > 1.5:
                break
        h = float(rng.uniform(1.2, 2.8))
        if i % 2 == 0:
            sx = float(rng.uniform(0.18, 0.45))
            sy = float(rng.uniform(0.18, 0.45))
            objs.append(SceneObject("obstacle_box%d" % i, "box",
                                    (x, y, h / 2), (sx, sy, h / 2),
                                    (0.55, 0.32, 0.25, 1.0)))
        else:
            r = float(rng.uniform(0.15, 0.35))
            objs.append(SceneObject("obstacle_cyl%d" % i, "cylinder",
                                    (x, y, h / 2), (r, h / 2),
                                    (0.35, 0.45, 0.35, 1.0)))
    return Scene(name="safety_field", objects=objs,
                 notes="安全过滤器场景：%d 个高 ≥1.2 m 的盒/柱，种子 %d。"
                       % (n_obstacles, seed))


@dataclass
class EnvConfig:
    scene: str = "safety_field"
    n_obstacles: int = 20
    area: float = 9.0
    max_time: float = 30.0
    speed: float = 2.0
    max_horiz: float = 4.0
    max_vert: float = 2.0
    max_yaw_rate: float = 2.5
    repulse: float = 1.2          # 标称控制器的斥力增益
    repulse_range: float = 1.8
    range_cfg: RangeConfig = field(default_factory=RangeConfig)

    def label(self) -> str:
        return "%s/area%.1f/drone%.2f" % (self.scene, self.area, 0.16)


def _opposite_points(rng: np.random.Generator, scene, radius: float,
                     z: float) -> Tuple[np.ndarray, np.ndarray]:
    for _ in range(200):
        th = rng.uniform(0.0, 2 * np.pi)
        r1 = rng.uniform(radius - 1.0, radius)
        r2 = rng.uniform(radius - 1.0, radius)
        start = np.array([r1 * math.cos(th), r1 * math.sin(th),
                          z + rng.uniform(-0.3, 0.3)])
        goal = np.array([-r2 * math.cos(th), -r2 * math.sin(th),
                         z + rng.uniform(-0.3, 0.3)])
        v0 = (goal - start) / max(np.linalg.norm(goal - start), 1e-6) * 2.0
        if (clearance_at(scene, start) > 0.8 and clearance_at(scene, goal) > 0.5
                and swept_clearance(scene, start, v0) > 1.0):
            return start, goal
    return np.array([-radius, 0.0, z]), np.array([radius, 0.0, z])


def _near_obstacle_point(rng: np.random.Generator, scene, z: float) -> np.ndarray:
    objs = [o for o in scene.objects if o.kind in ("box", "cylinder", "sphere")]
    for _ in range(200):
        o = objs[int(rng.integers(0, len(objs)))]
        ang = rng.uniform(0.0, 2 * np.pi)
        dist = rng.uniform(0.5, 1.0)
        p = np.array([o.pos[0] + dist * math.cos(ang),
                      o.pos[1] + dist * math.sin(ang),
                      z + rng.uniform(-0.2, 0.2)])
        if 0.25 < clearance_at(scene, p) < 1.0:
            return p
    return np.array([0.0, 0.0, z])


def make_episodes(seeds: Sequence[int], per_seed: int = 1,
                  near_frac: float = 0.0, radius: float = 7.5,
                  z: float = 1.6, n_obstacles: int = 20) -> List[EpisodeSpec]:
    out: List[EpisodeSpec] = []
    for seed in seeds:
        scene = safety_field(seed=seed, n_obstacles=n_obstacles, area=9.0)
        rng = np.random.default_rng(1000 + seed)
        for k in range(per_seed):
            if near_frac > 0 and rng.random() < near_frac:
                start = _near_obstacle_point(rng, scene, z)
                for _ in range(100):
                    ang = math.atan2(-start[1], -start[0]) + rng.uniform(-0.5, 0.5)
                    goal = np.array([radius * math.cos(ang),
                                     radius * math.sin(ang),
                                     z + rng.uniform(-0.3, 0.3)])
                    if clearance_at(scene, goal) > 0.5:
                        break
            else:
                start, goal = _opposite_points(rng, scene, radius, z)
            out.append(EpisodeSpec(seed, start, goal, "s%d/e%d" % (seed, k)))
    return out


class SafetyEnv:
    def __init__(self, cfg: Optional[EnvConfig] = None):
        self.cfg = cfg or EnvConfig()
        self.scene = self._build_scene(0)
        self.quad = Quadrotor(scene=self.scene, dt=SIM_DT, substeps=4)
        self.ctrl = GeometricController(self.quad.params)
        self.lidar = LidarSensor(self.quad.model, self.cfg.range_cfg)
        self._sub = int(round(FRAME_DT / SIM_DT))
        self._v_filt = np.zeros(3)
        self._ref = np.zeros(3)
        self._ref_yaw = 0.0
        self._t = 0.0
        self._path = 0.0
        self.spec: Optional[EpisodeSpec] = None
        self.goal = np.zeros(3)
        self._grid = np.full((3, 36), MAX_RANGE, np.float32)
        self.h = 0.0
        self.collided = False
        self.success = False

    # ------------------------------------------------------------------ 生命周期
    def reset(self, spec: EpisodeSpec) -> Dict[str, np.ndarray]:
        self.spec = spec
        self.goal = np.asarray(spec.goal, float).copy()
        p = np.asarray(spec.start, float)
        yaw = math.atan2(self.goal[1] - p[1], self.goal[0] - p[0])
        self.quad.reset(pos=tuple(p), yaw=yaw)
        self._v_filt = np.zeros(3)
        self._ref = p.copy()
        self._ref_yaw = float(yaw)
        self._t = 0.0
        self._path = 0.0
        self.collided = False
        self.success = False
        self.h = clearance_at(self.scene, p)
        return self.observe()

    def _build_scene(self, seed: int) -> Scene:
        if self.cfg.scene == "safety_field":
            return safety_field(seed=seed, n_obstacles=self.cfg.n_obstacles,
                                area=self.cfg.area)
        return make_scene(self.cfg.scene, seed=seed,
                          n_obstacles=self.cfg.n_obstacles,
                          area=self.cfg.area)

    def reseed_scene(self, seed: int) -> None:
        """换场景（不同种子）并保留同一个 ``Quadrotor`` 实例。"""
        self.scene = self._build_scene(seed)
        self.quad = Quadrotor(scene=self.scene, dt=SIM_DT, substeps=4)
        self.ctrl = GeometricController(self.quad.params)
        self.lidar = LidarSensor(self.quad.model, self.cfg.range_cfg)

    # ------------------------------------------------------------------ 观测
    def observe(self) -> Dict[str, np.ndarray]:
        st = self.quad.state()
        grid = scan_grid(self.lidar, self.quad.data, st.pos, self._ref_yaw)
        self._grid = grid
        c, s = math.cos(self._ref_yaw), math.sin(self._ref_yaw)
        Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        vel = Rz.T @ st.vel
        dp = Rz.T @ (self.goal - st.pos)
        dist = float(np.linalg.norm(dp))
        goal_dir = dp / max(dist, 1e-6)
        feat = build_features(grid, vel, goal_dir, dist, float(st.pos[2]))
        return {"ranges": grid.astype(np.float32),
                "feat": feat.astype(np.float32),
                "vel": vel.astype(np.float32),
                "goal_dir": goal_dir.astype(np.float32),
                "dist": np.float32(dist),
                "pos": st.pos.astype(np.float32),
                "yaw": np.float32(self._ref_yaw),
                "h": np.float32(rollout_clearance_many(
                    self.scene, st.pos, st.vel[None], self._ref_yaw,
                    vel[None])[0])}

    def nominal_action(self) -> np.ndarray:
        """目标吸引 + 距离栅格斥力，输出机头系速度；不保证安全。"""
        st = self.quad.state()
        c, s = math.cos(self._ref_yaw), math.sin(self._ref_yaw)
        Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        g = Rz.T @ (self.goal - st.pos)
        d = max(float(np.hypot(g[0], g[1])), 1e-6)
        fwd = np.array([g[0], g[1]]) / d
        rng = np.min(self._grid, axis=0)
        w = np.clip((self.cfg.repulse_range - rng)
                    / max(self.cfg.repulse_range, 1e-6), 0.0, 1.0) ** 2
        az = -np.pi + 2 * np.pi * np.arange(len(rng)) / len(rng)
        rep = np.array([float(np.sum(w * np.cos(az))),
                        float(np.sum(w * np.sin(az)))])
        v = fwd + self.cfg.repulse * rep
        n = float(np.linalg.norm(v))
        if n > 1e-6:
            v = v / n
        vz = float(np.clip(0.8 * (self.goal[2] - st.pos[2]), -0.8, 0.8))
        return np.array([self.cfg.speed * v[0], self.cfg.speed * v[1], vz])

    # ------------------------------------------------------------------ 步进
    def _command(self, action: np.ndarray) -> np.ndarray:
        cfg = self.cfg
        a = np.asarray(action, float).reshape(3)
        a = np.array([np.clip(a[0], -1.0, cfg.max_horiz),
                      np.clip(a[1], -3.0, 3.0),
                      np.clip(a[2], -cfg.max_vert, cfg.max_vert)])
        c, s = math.cos(self._ref_yaw), math.sin(self._ref_yaw)
        Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        v_world = Rz @ a
        tau = 0.12
        alpha = min(1.0, FRAME_DT / tau)
        self._v_filt = self._v_filt + (v_world - self._v_filt) * alpha
        self._ref = self._ref + self._v_filt * FRAME_DT
        self._ref[2] = float(np.clip(self._ref[2], 0.5, 8.0))
        vh = self._v_filt[:2]
        if np.linalg.norm(vh) > 0.4:
            want = math.atan2(vh[1], vh[0])
            d = math.atan2(math.sin(want - self._ref_yaw),
                           math.cos(want - self._ref_yaw))
            lim = cfg.max_yaw_rate * FRAME_DT
            self._ref_yaw += max(-lim, min(lim, d))
        return a

    def step(self, action: np.ndarray):
        a = self._command(action)
        p_prev = self.quad.state().pos.copy()
        for _ in range(self._sub):
            self.quad.step_with_control(
                self.ctrl.act(self.quad, self._ref, self._v_filt,
                              np.zeros(3), self._ref_yaw))
        self._t += FRAME_DT
        st = self.quad.state()
        p = st.pos.copy()
        self._path += float(np.linalg.norm(p - p_prev))
        self.h = clearance_at(self.scene, p)
        self.collided = self.h < 0.0
        self.success = float(np.linalg.norm(p - self.goal)) < GOAL_RADIUS
        done = self.collided or self.success or self._t >= self.cfg.max_time
        obs = self.observe()
        info = {"h": float(self.h), "collision": bool(self.collided),
                "success": bool(self.success), "time": float(self._t),
                "path": float(self._path), "pos": p.copy(),
                "dist": float(np.linalg.norm(p - self.goal)),
                "min_range": grid_min_range(obs["ranges"]),
                "speed": float(np.linalg.norm(st.vel))}
        return obs, info, bool(done)

    def close(self) -> None:
        pass

    def __del__(self):
        try:
            self.close()
        except Exception:                                       # noqa: BLE001
            pass
