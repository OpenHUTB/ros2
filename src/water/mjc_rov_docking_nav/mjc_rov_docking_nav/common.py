"""common.py — 路径与声呐扫描工具。"""
import os
from pathlib import Path
import numpy as np


def get_package_root() -> Path:
    return Path(__file__).resolve().parent.parent


def find_weights_path(explicit: str = "") -> Path:
    if explicit:
        return Path(explicit)
    return get_package_root() / "weights" / "dock_cnn.pt"


def set_mujoco_gl():
    os.environ.setdefault("MUJOCO_GL", "osmesa")
    os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")


def sonar_scan(rov_xy: np.ndarray, dock_xy: np.ndarray, obs_list: list,
               n_bins: int = 32, max_range: float = 5.0) -> np.ndarray:
    """
    模拟前视声呐扫描：返回 n_bins 个距离值（有障碍物=距离，无=max_range）。
    为什么这么做：真实前视声呐输出扇形距离剖面，这里简化为一维距离数组，
    作为 CNN 输入判断"前方是否有对接标记"。
    """
    ranges = np.full(n_bins, max_range, dtype=np.float32)
    angles = np.linspace(-np.pi/3, np.pi/3, n_bins)
    for i, ang in enumerate(angles):
        # 检查对接标记
        dx = dock_xy[0] - rov_xy[0]
        dy = dock_xy[1] - rov_xy[1]
        dist = np.hypot(dx, dy)
        ang_to_dock = np.arctan2(dy, dx)
        ang_diff = abs(ang_to_dock - ang)
        if ang_diff < 0.15 and dist < max_range:
            ranges[i] = dist * 0.9  # 对接标记回波强，距离略近
        # 检查障碍物
        for obs in obs_list:
            dxo = obs[0] - rov_xy[0]
            dyo = obs[1] - rov_xy[1]
            dist_o = np.hypot(dxo, dyo)
            ang_o = np.arctan2(dyo, dxo)
            if abs(ang_o - ang) < 0.15 and dist_o < ranges[i]:
                ranges[i] = dist_o
    return ranges
