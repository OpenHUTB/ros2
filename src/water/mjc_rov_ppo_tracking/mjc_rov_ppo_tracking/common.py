"""common.py — 路径与环境工具。"""
import os
from pathlib import Path
import numpy as np


def get_package_root() -> Path:
    return Path(__file__).resolve().parent.parent


def find_weights_path(explicit: str = "") -> Path:
    if explicit:
        return Path(explicit)
    return get_package_root() / "weights" / "ppo_actor.pt"


def set_mujoco_gl():
    os.environ.setdefault("MUJOCO_GL", "osmesa")
    os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")


def pid_control(error: np.ndarray, prev_error: np.ndarray, integral: np.ndarray,
               kp: float = 1.0, ki: float = 0.0, kd: float = 0.1, dt: float = 0.02):
    """二维 PID 控制器，返回推力 [Fx, Fy]。"""
    integral += error * dt
    derivative = (error - prev_error) / dt
    out = kp * error + ki * integral + kd * derivative
    return out, integral, error
