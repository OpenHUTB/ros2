#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单门深度环境：深度观测、门中心标签、固定航向比例伺服。"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import numpy as np

from uav_mujoco_sim.core import GeometricController, Quadrotor
from uav_mujoco_sim.core.scenes import make_scene
from uav_mujoco_sim.core.sensors import CameraConfig, VisionSensors

from .depth import DEPTH_HW, NET_HW, downsample_depth, valid_fraction
from .geometry import CAM_OFFSET_BODY, camera_label, quat_to_mat
from .servo import GateServo, ServoConfig

__all__ = ["GateSpec", "parse_single_gate", "EnvConfig", "GateServoEnv"]

SIM_DT = 0.01
FRAME_DT = 0.05


@dataclass
class GateSpec:
    x: float
    y: float
    z: float
    half: float = 1.1

    @property
    def center(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=np.float64)

    def contains(self, p: np.ndarray) -> bool:
        return abs(float(p[1]) - self.y) <= self.half \
            and abs(float(p[2]) - self.z) <= self.half


def parse_single_gate(scene) -> GateSpec:
    """从单门 scene 的两根 gate_post 解析中心。"""
    posts = [o for o in scene.objects if o.name.startswith("gate_post")]
    if len(posts) != 2:
        raise ValueError("单门场景应有两根 gate_post，实际 %d 根" % len(posts))
    x = float(np.mean([o.pos[0] for o in posts]))
    y = float(np.mean([o.pos[1] for o in posts]))
    z = float(np.mean([o.pos[2] for o in posts]))
    return GateSpec(x=x, y=y, z=z, half=1.1)


@dataclass
class EnvConfig:
    scene: str = "gate_track"
    scene_seed: int = 16
    max_time: float = 12.0
    dt: float = SIM_DT
    frame_dt: float = FRAME_DT
    substeps: int = 4
    depth_hw: Tuple[int, int] = DEPTH_HW
    net_hw: Tuple[int, int] = NET_HW
    depth_max_range: float = 10.0
    start_dist: Tuple[float, float] = (2.0, 6.0)
    start_lateral: float = 1.5
    start_vertical: float = 0.8
    tau: float = 0.18
    max_horiz: float = 2.0
    max_vert: float = 1.2
    servo: ServoConfig = field(default_factory=ServoConfig)


class GateServoEnv:
    """一个 episode = 从随机起点穿过一个门。"""

    def __init__(self, cfg: Optional[EnvConfig] = None):
        self.cfg = cfg or EnvConfig()
        self.scene = make_scene(self.cfg.scene, seed=self.cfg.scene_seed,
                                n_gates=1)
        self.gate = parse_single_gate(self.scene)
        self.quad = Quadrotor(scene=self.scene, dt=self.cfg.dt,
                              substeps=self.cfg.substeps)
        self.ctrl = GeometricController(self.quad.params)
        self.cam = CameraConfig(width=self.cfg.depth_hw[1],
                                height=self.cfg.depth_hw[0], quality="fast",
                                depth_max_range=self.cfg.depth_max_range)
        self.vision: Optional[VisionSensors] = None
        self._servo = GateServo(self.cfg.servo)
        self._sub = int(round(self.cfg.frame_dt / self.cfg.dt))
        self._v_filt = np.zeros(3)
        self._ref = np.zeros(3)
        self._t = 0.0
        self._done = False
        self._reason = "running"
        self._info: Dict[str, object] = {}
        self._prev_pos = np.zeros(3)

    def _ensure_vision(self) -> None:
        if self.vision is None:
            self.vision = VisionSensors(self.quad.model, self.cam)

    @property
    def servo(self) -> GateServo:
        return self._servo

    def raw_depth(self) -> np.ndarray:
        self._ensure_vision()
        return self.vision.depth(self.quad.data)

    def reset(self, seed: Optional[int] = None,
              distance: Optional[float] = None,
              lateral: Optional[float] = None,
              vertical: Optional[float] = None) -> Dict[str, np.ndarray]:
        rng = np.random.default_rng(0 if seed is None else int(seed))
        d = float(distance) if distance is not None \
            else float(rng.uniform(*self.cfg.start_dist))
        lat = float(lateral) if lateral is not None \
            else float(rng.uniform(-self.cfg.start_lateral, self.cfg.start_lateral))
        vert = float(vertical) if vertical is not None \
            else float(rng.uniform(-self.cfg.start_vertical, self.cfg.start_vertical))
        pos = np.array([self.gate.x - d, self.gate.y + lat, self.gate.z + vert])
        pos[2] = float(np.clip(pos[2], 0.8, 3.6))
        self.quad.reset(pos=tuple(pos), yaw=0.0)
        self._v_filt = np.zeros(3)
        self._ref = pos.copy()
        self._t = 0.0
        self._done = False
        self._reason = "running"
        self._info = {}
        self._prev_pos = pos.copy()
        return self.observe()

    def observe(self) -> Dict[str, np.ndarray]:
        self._ensure_vision()
        st = self.quad.state()
        raw = self.vision.depth(self.quad.data)
        depth = downsample_depth(raw, self.cfg.net_hw)
        label = camera_label(self.gate.center, st.pos, st.quat)
        return {"depth": depth.astype(np.float32),
                "label": label.astype(np.float32),
                "valid_frac": np.float32(valid_fraction(raw)),
                "pos": st.pos.astype(np.float32),
                "quat": st.quat.astype(np.float32),
                "yaw": np.float32(st.yaw)}

    def oracle_command(self) -> np.ndarray:
        st = self.quad.state()
        label = camera_label(self.gate.center, st.pos, st.quat)
        offset = quat_to_mat(st.quat) @ CAM_OFFSET_BODY
        return self.servo.command(label, offset, 1.0, True)

    def _command(self, action: np.ndarray) -> np.ndarray:
        """航向系速度（m/s）→ 参考位置；固定航向，不做偏航跟随。"""
        a = np.asarray(action, dtype=float).reshape(3)
        a = np.array([np.clip(a[0], -self.cfg.max_horiz, self.cfg.max_horiz),
                      np.clip(a[1], -self.cfg.max_horiz, self.cfg.max_horiz),
                      np.clip(a[2], -self.cfg.max_vert, self.cfg.max_vert)])
        alpha = min(1.0, self.cfg.frame_dt / max(self.cfg.tau, 1e-6))
        self._v_filt = self._v_filt + (a - self._v_filt) * alpha
        self._ref = self._ref + self._v_filt * self.cfg.frame_dt
        self._ref[2] = float(np.clip(self._ref[2], 0.6, 9.0))
        return a

    def step(self, action: np.ndarray):
        if self._done:
            return self.observe(), True, self._info
        self._command(action)

        p_prev = self.quad.state().pos.copy()
        for _ in range(self._sub):
            self.quad.step_with_control(
                self.ctrl.act(self.quad, self._ref, self._v_filt,
                              np.zeros(3), 0.0))
        self._t += self.cfg.frame_dt
        st = self.quad.state()
        p = st.pos.copy()

        if p_prev[0] < self.gate.x <= p[0]:
            self._reason = "pass" if self.gate.contains(p) else "miss"
            self._done = True
        elif self._t >= self.cfg.max_time:
            self._reason = "timeout"
            self._done = True

        self._info = {
            "reason": self._reason,
            "passed": self._reason == "pass",
            "missed": self._reason == "miss",
            "timeout": self._reason == "timeout",
            "time": float(self._t),
            "pos": p.copy(),
            "yaw": float(st.yaw),
            "distance": float(np.linalg.norm(self.gate.center - p)),
        }
        return self.observe(), bool(self._done), self._info

    def close(self) -> None:
        if self.vision is not None:
            self.vision.close()
            self.vision = None

    def __del__(self):
        try:
            self.close()
        except Exception:  # noqa: BLE001
            pass
