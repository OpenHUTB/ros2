#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""动态障碍场上的安全飞行环境。

和 ``uav_mujoco_sim`` 的其它场景一样：动作是机头系速度指令，底层由几何
控制器跟踪；障碍在仿真里是真实的 MuJoCo geom，运动障碍每帧改写
``model.geom_pos`` 后 ``mj_forward``，因此激光、碰撞与间距三者一致。

与静态版本的区别只有两点：场景里有绕圈运动的高柱，观测是三帧距离栅格。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

from uav_mujoco_sim.core import GeometricController, Quadrotor
from uav_mujoco_sim.core.scenes import Scene, SceneObject
from uav_mujoco_sim.core.sensors import LidarSensor

from .field import DynamicField, safety_field, sample_endpoints
from .geometry import clearance_at, rollout_clearance
from .ranges import (FEAT_DIM, FEAT_DIM_SEQ, HISTORY, MAX_RANGE, RangeConfig,
                     build_features, build_sequence_features, grid_min_range,
                     scan_grid)

__all__ = ["SIM_DT", "CTRL_HZ", "FRAME_DT", "GOAL_RADIUS", "EpisodeSpec",
           "EnvConfig", "make_episodes", "DynamicEnv"]

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
    n_moving: int = 5

    def key(self) -> str:
        return self.label or "seed%d" % self.seed


@dataclass
class EnvConfig:
    n_static: int = 18
    n_moving: int = 5
    area: float = 9.0
    max_time: float = 20.0
    speed: float = 1.8
    max_horiz: float = 4.0
    max_vert: float = 2.0
    max_yaw_rate: float = 1.5
    acc_max: float = 4.0            # 参考速度的变化率上限（m/s^2）
    # 姿态环增益：底座默认 (kr=-1.5, kw=-0.10) 在 2 m/s 级反复改向时会出现
    # 极限环（roll +75° → -83°），姿态发散后升力骤降。这里加阻尼。
    ctrl_kr: tuple = (-3.0, -3.0, -1.2)
    ctrl_kw: tuple = (-0.25, -0.25, -0.15)
    repulse: float = 1.1
    repulse_range: float = 1.8
    mover_speed: tuple = (0.4, 1.0)
    range_cfg: RangeConfig = field(default_factory=RangeConfig)

    def label(self) -> str:
        return "static%d/moving%d/area%.1f" % (self.n_static, self.n_moving,
                                               self.area)


def make_episodes(seeds: Sequence[int], per_seed: int = 1, radius: float = 7.5,
                  z: float = 1.6, cfg: Optional[EnvConfig] = None,
                  n_moving: Optional[int] = None) -> List[EpisodeSpec]:
    cfg = cfg or EnvConfig()
    out: List[EpisodeSpec] = []
    nm = cfg.n_moving if n_moving is None else int(n_moving)
    for seed in seeds:
        fld = safety_field(seed=seed, n_static=cfg.n_static, n_moving=nm,
                           area=cfg.area, mover_speed=cfg.mover_speed)
        rng = np.random.default_rng(2026 + int(seed))
        for k in range(int(per_seed)):
            start, goal = sample_endpoints(fld, rng, radius=radius, z=z)
            out.append(EpisodeSpec(int(seed), start, goal,
                                   "s%d/e%d" % (seed, k), nm))
    return out


def _scene_from_field(fld: DynamicField, label: str) -> Scene:
    """把 ``DynamicField`` 转成底座认识的 ``Scene``（运动障碍放在初始位置）。"""
    objs: List[SceneObject] = []
    for i, (kind, pos, size) in enumerate(fld.static):
        name = "static_%s%d" % (kind[:3], i)
        if kind == "box":
            objs.append(SceneObject(name, "box", tuple(pos), tuple(size),
                                    (0.55, 0.32, 0.25, 1.0)))
        else:
            objs.append(SceneObject(name, "cylinder", tuple(pos), tuple(size),
                                    (0.35, 0.45, 0.35, 1.0)))
    for m in fld.movers:
        objs.append(SceneObject(m.name, "cylinder", tuple(m.pos_at(0.0)),
                                tuple(m.size), (0.85, 0.25, 0.25, 1.0)))
    return Scene(name="dynamic_field", objects=objs,
                 ground_size=(70.0, 70.0),
                 notes="动态障碍场：%s（%s）" % (fld.label(), label))


class DynamicEnv:
    """MuJoCo 四旋翼 + 动态障碍场，20 Hz 决策。"""

    def __init__(self, cfg: Optional[EnvConfig] = None):
        self.cfg = cfg or EnvConfig()
        self._sub = int(round(FRAME_DT / SIM_DT))
        self.field = safety_field(seed=0, n_static=self.cfg.n_static,
                                  n_moving=self.cfg.n_moving,
                                  area=self.cfg.area,
                                  mover_speed=self.cfg.mover_speed)
        self._build(self.field)
        self.spec: Optional[EpisodeSpec] = None
        self.goal = np.zeros(3)
        self._v_filt = np.zeros(3)
        self._ref = np.zeros(3)
        self._ref_yaw = 0.0
        self._t = 0.0
        self._path = 0.0
        self._hist: List[np.ndarray] = []
        self._grid = np.full((3, 36), MAX_RANGE, np.float32)
        self.h = 0.0
        self.collided = False
        self.success = False

    # ------------------------------------------------------------ 构建
    def _build(self, fld: DynamicField) -> None:
        self.field = fld
        scene = _scene_from_field(fld, self.cfg.label())
        self.quad = Quadrotor(scene=scene, dt=SIM_DT, substeps=4)
        self.ctrl = GeometricController(self.quad.params,
                                        kr=self.cfg.ctrl_kr,
                                        kw=self.cfg.ctrl_kw)
        self.lidar = LidarSensor(self.quad.model, self.cfg.range_cfg)
        self._mover_geoms = []
        import mujoco
        for m in fld.movers:
            gid = mujoco.mj_name2id(self.quad.model, mujoco.mjtObj.mjOBJ_GEOM,
                                    m.name)
            if gid >= 0:
                self._mover_geoms.append((m, gid))
        self._sync_movers(0.0)

    def _sync_movers(self, t: float) -> None:
        if not self._mover_geoms:
            return
        import mujoco
        model = self.quad.model
        for m, gid in self._mover_geoms:
            model.geom_pos[gid] = m.pos_at(t)
        mujoco.mj_forward(model, self.quad.data)

    def reseed_scene(self, seed: int, n_moving: Optional[int] = None) -> None:
        nm = self.cfg.n_moving if n_moving is None else int(n_moving)
        self._build(safety_field(seed=int(seed), n_static=self.cfg.n_static,
                                 n_moving=nm, area=self.cfg.area,
                                 mover_speed=self.cfg.mover_speed))

    # ------------------------------------------------------------ 生命周期
    def reset(self, spec: EpisodeSpec) -> Dict[str, np.ndarray]:
        if self.field.seed != int(spec.seed) or \
                self.field.n_moving() != int(spec.n_moving):
            self.reseed_scene(int(spec.seed), int(spec.n_moving))
        self.spec = spec
        self.goal = np.asarray(spec.goal, float).copy()
        p = np.asarray(spec.start, float)
        yaw = math.atan2(self.goal[1] - p[1], self.goal[0] - p[0])
        self.quad.reset(pos=tuple(p), yaw=float(yaw))
        self._v_filt = np.zeros(3)
        self._ref = p.copy()
        self._ref_yaw = float(yaw)
        self._t = 0.0
        self._path = 0.0
        self.collided = False
        self.success = False
        self._sync_movers(0.0)
        self.h = self.field.clearance_at(p, 0.0)
        obs = self.observe()
        self._hist = [obs["ranges"].copy() for _ in range(HISTORY)]
        return self.observe()

    # ------------------------------------------------------------ 观测
    def _rot(self) -> np.ndarray:
        c, s = math.cos(self._ref_yaw), math.sin(self._ref_yaw)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    def observe(self) -> Dict[str, np.ndarray]:
        st = self.quad.state()
        grid = scan_grid(self.lidar, self.quad.data, st.pos, self._ref_yaw)
        self._grid = grid
        rot = self._rot()
        vel = rot.T @ st.vel
        dp = rot.T @ (self.goal - st.pos)
        dist = float(np.linalg.norm(dp))
        goal_dir = dp / max(dist, 1e-6)
        hist = [grid] + [g for g in self._hist[-HISTORY + 1:]]
        hist = hist[-HISTORY:]
        while len(hist) < HISTORY:
            hist.insert(0, grid)
        feat_s = build_features(grid, vel, goal_dir, dist, float(st.pos[2]))
        feat = build_sequence_features(np.asarray(hist, np.float32), vel,
                                       goal_dir, dist, float(st.pos[2]))
        h = float(rollout_clearance(self.field.batch_at, st.pos,
                                    st.vel[None], self._ref_yaw,
                                    vel[None], horizon=0.6,
                                    max_yaw_rate=self.cfg.max_yaw_rate,
                                    acc_max=self.cfg.acc_max, t0=self._t)[0])
        return {"ranges": grid.astype(np.float32),
                "hist": np.asarray(hist, np.float32),
                "feat": feat.astype(np.float32),
                "feat_single": feat_s.astype(np.float32),
                "vel": vel.astype(np.float32),
                "goal_dir": goal_dir.astype(np.float32),
                "dist": np.float32(dist),
                "pos": st.pos.astype(np.float32),
                "yaw": np.float32(self._ref_yaw),
                "t": np.float32(self._t),
                "h": np.float32(h),
                "movers": self.field.mover_positions(self._t).astype(np.float32)}

    def nominal_action(self) -> np.ndarray:
        """目标吸引 + 距离栅格斥力，输出机头系速度；不考虑障碍运动。"""
        st = self.quad.state()
        rot = self._rot()
        g = rot.T @ (self.goal - st.pos)
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

    # ------------------------------------------------------------ 步进
    def _command(self, action: np.ndarray) -> np.ndarray:
        cfg = self.cfg
        a = np.asarray(action, float).reshape(3)
        a = np.array([np.clip(a[0], -1.5, cfg.max_horiz),
                      np.clip(a[1], -3.0, 3.0),
                      np.clip(a[2], -cfg.max_vert, cfg.max_vert)])
        v_world = self._rot() @ a
        tau = 0.12
        alpha = min(1.0, FRAME_DT / tau)
        dv = (v_world - self._v_filt) * alpha
        lim = float(cfg.acc_max) * FRAME_DT
        if lim > 0:
            dv = np.clip(dv, -lim, lim)
        self._v_filt = self._v_filt + dv
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
        t_next = self._t + FRAME_DT
        for _ in range(self._sub):
            self.quad.step_with_control(
                self.ctrl.act(self.quad, self._ref, self._v_filt,
                              np.zeros(3), self._ref_yaw))
        self._t = t_next
        self._sync_movers(self._t)
        st = self.quad.state()
        p = st.pos.copy()
        self._path += float(np.linalg.norm(p - p_prev))
        self.h = self.field.clearance_at(p, self._t)
        self.collided = bool(self.h < 0.0)
        self.success = bool(float(np.linalg.norm(p - self.goal)) < GOAL_RADIUS)
        out_of_bounds = not self.field.in_bounds(p, margin=2.0)
        done = self.collided or self.success or out_of_bounds or \
            self._t >= self.cfg.max_time
        self._hist.append(self._grid.copy())
        self._hist = self._hist[-HISTORY:]
        obs = self.observe()
        info = {"h": float(self.h), "collision": bool(self.collided),
                "success": bool(self.success), "time": float(self._t),
                "path": float(self._path), "pos": p.copy(),
                "dist": float(np.linalg.norm(p - self.goal)),
                "min_range": grid_min_range(obs["ranges"]),
                "speed": float(np.linalg.norm(st.vel)),
                "out_of_bounds": bool(out_of_bounds),
                "action": a.copy(),
                "t": float(self._t)}
        return obs, info, bool(done)

    def close(self) -> None:
        pass

    def __del__(self):                                   # pragma: no cover
        try:
            self.close()
        except Exception:                                # noqa: BLE001
            pass
