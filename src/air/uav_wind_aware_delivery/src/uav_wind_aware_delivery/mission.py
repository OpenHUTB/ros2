# -*- coding: utf-8 -*-
"""空投任务状态机：起飞 → 巡航 → 对准 → 释放 → 落地评估。"""
from __future__ import annotations

import os
from dataclasses import dataclass, field

import numpy as np

from .ballistics import simulate_drop
from .nn import ImpactMLP


@dataclass
class MissionConfig:
    target: np.ndarray = field(default_factory=lambda: np.array([5.0, 2.0, 0.0]))
    release_alt: float = 3.0
    hover_time: float = 0.9
    compensate: bool = True
    max_time: float = 45.0


class DeliveryMission:
    def __init__(self, cfg: MissionConfig | None = None, model_path: str | None = None):
        self.cfg = cfg or MissionConfig()
        self.model = None
        if model_path and os.path.exists(model_path):
            try:
                self.model = ImpactMLP.load(model_path)
            except Exception:
                self.model = None
        self.phase = "takeoff"
        self.phase_t = 0.0
        self.release_sent = False
        self.done = False
        self.rel = np.asarray(self.cfg.target[:2], dtype=float).copy()
        self.predicted_offset = np.zeros(2)
        self.predicted_landing = self.rel.copy()
        self.landing_error = float("nan")
        self._last_t = 0.0
        self.start_xy = None

    # ------------------------------------------------------------ predictor
    def predict_offset(self, h: float, vx: float, vy: float, vz: float,
                       wind: np.ndarray) -> np.ndarray:
        if self.model is not None:
            x = np.array([[h, vx, vy, vz, wind[0], wind[1]]], dtype=float)
            out = self.model.predict(x)[0]
            if np.all(np.isfinite(out)):
                return out
        return simulate_drop(h, vx, vy, vz, wind)

    def compute_release_point(self, wind: np.ndarray) -> np.ndarray:
        if not self.cfg.compensate:
            return np.asarray(self.cfg.target[:2], dtype=float).copy()
        rel = np.asarray(self.cfg.target[:2], dtype=float).copy()
        off = np.zeros(2)
        for _ in range(4):
            off = self.predict_offset(self.cfg.release_alt, 0.0, 0.0, 0.0, wind)
            rel = np.asarray(self.cfg.target[:2], dtype=float) - off
        self.predicted_offset = off
        self.predicted_landing = np.asarray(self.cfg.target[:2], dtype=float)
        return rel

    # ---------------------------------------------------------------- state
    def update(self, st) -> dict:
        dt = max(0.0, st.t - self._last_t)
        self._last_t = st.t
        if self.done:
            return self._out(np.array([st.pos[0], st.pos[1], self.cfg.release_alt]), False)

        self.phase_t += dt
        if self.start_xy is None:
            self.start_xy = st.pos[:2].copy()
        self.rel = self.compute_release_point(st.wind)
        target = self.cfg.target[:2]

        if self.phase == "takeoff":
            # 关键：起飞阶段固定水平目标；若跟随当前 xy，风会把无人机一路吹走
            sp = np.array([self.start_xy[0], self.start_xy[1], self.cfg.release_alt])
            if abs(st.pos[2] - self.cfg.release_alt) < 0.16 and np.linalg.norm(st.vel) < 0.35:
                self._next("cruise")
        elif self.phase == "cruise":
            sp = np.array([self.rel[0], self.rel[1], self.cfg.release_alt])
            horiz = np.linalg.norm(st.pos[:2] - self.rel)
            if horiz < 0.22 and np.linalg.norm(st.vel) < 0.30:
                self._next("align")
        elif self.phase == "align":
            sp = np.array([self.rel[0], self.rel[1], self.cfg.release_alt])
            if self.phase_t >= self.cfg.hover_time and np.linalg.norm(st.vel) < 0.25:
                self.release_sent = True
                self._next("released")
        elif self.phase == "released":
            sp = np.array([self.rel[0], self.rel[1], self.cfg.release_alt])
            if st.landed:
                self.landing_error = float(np.linalg.norm(st.payload_pos[:2] - target))
                self._next("done")
                self.done = True
        else:
            sp = np.array([st.pos[0], st.pos[1], self.cfg.release_alt])

        if st.t > self.cfg.max_time and not self.done:
            if st.released:
                self.landing_error = float(np.linalg.norm(st.payload_pos[:2] - target))
            self.done = True
            self.phase = "timeout"
        return self._out(sp, self.release_sent)

    def _next(self, phase: str):
        self.phase = phase
        self.phase_t = 0.0

    def _out(self, sp, release):
        return {
            "setpoint": np.asarray(sp, dtype=float),
            "release": bool(release),
            "phase": self.phase,
            "release_point": self.rel.copy(),
            "predicted_offset": self.predicted_offset.copy(),
            "predicted_landing": self.predicted_landing.copy(),
            "landing_error": self.landing_error,
            "done": self.done,
        }
