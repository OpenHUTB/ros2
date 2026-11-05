# -*- coding: utf-8 -*-
"""无 MuJoCo 依赖的状态对象与 ROS 向量编解码。"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class State:
    t: float
    pos: np.ndarray
    quat: np.ndarray
    vel: np.ndarray
    omega: np.ndarray
    euler: np.ndarray
    payload_pos: np.ndarray
    payload_vel: np.ndarray
    released: bool
    landed: bool
    payload_contact: bool
    wind: np.ndarray

    def as_vector(self) -> np.ndarray:
        """固定 28 维：t,pos,quat,vel,omega,euler,payload_pos,payload_vel,flags,wind。"""
        return np.concatenate([
            [self.t], self.pos, self.quat, self.vel, self.omega, self.euler,
            self.payload_pos, self.payload_vel,
            [float(self.released), float(self.landed), float(self.payload_contact)],
            self.wind[:2],
        ]).astype(float)

    @classmethod
    def from_vector(cls, v) -> "State":
        v = np.asarray(v, dtype=float).ravel()
        if v.size < 28:
            raise ValueError(f"state vector expects 28 elements, got {v.size}")
        return cls(
            t=float(v[0]),
            pos=v[1:4].copy(),
            quat=v[4:8].copy(),
            vel=v[8:11].copy(),
            omega=v[11:14].copy(),
            euler=v[14:17].copy(),
            payload_pos=v[17:20].copy(),
            payload_vel=v[20:23].copy(),
            released=bool(v[23] > 0.5),
            landed=bool(v[24] > 0.5),
            payload_contact=bool(v[25] > 0.5),
            wind=np.array([v[26], v[27], 0.0]),
        )


# 兼容旧代码里的 typing 引用
__all__ = ["State"]
