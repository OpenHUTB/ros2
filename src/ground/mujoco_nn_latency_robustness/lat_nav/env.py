"""MuJoCo 赛道环境：速度伺服小车 + 24 线雷达 + 三条赛道。

物理用 slide/slide/hinge 三个关节 + velocity 执行器模拟"理想速度伺服"，
这样控制链路里的**延迟才是唯一的不确定性来源**，不会和轮胎模型混在一起。
"""
from __future__ import annotations

import zlib

import numpy as np

import mujoco

from . import track

DT = 0.05               # 控制周期 20 Hz
SUBSTEPS = 10           # 物理步长 0.005 s
LIDAR_BEAMS = 24
LIDAR_RANGE = 6.0
SENSOR_FOV = 2.0 * np.pi
ROBOT_RADIUS = 0.18


def _wrap(a: float) -> float:
    return (a + np.pi) % (2.0 * np.pi) - np.pi


class LatencyEnv:
    """一条赛道上的小车。step() 接受 (v, w) 并推进一个控制周期。"""

    SHAPES = track.SHAPES

    def __init__(self, shape: str = "circle", seed: int = 0, sensor_noise: float = 0.01):
        self.shape = shape
        self.xml = track.build_xml(shape)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.path = track.centerline(shape)
        self.sensor_noise = sensor_noise
        self._geomid = np.zeros(1, dtype=np.int32)
        self._bodyexclude = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "robot")
        self._geomgroup = np.ones(6, dtype=np.uint8)
        self._beam_angles = np.linspace(-np.pi, np.pi, LIDAR_BEAMS, endpoint=False)
        self.reset(seed=seed)

    # ---------------------------------------------------------------- 状态
    def reset(self, seed: int = 0) -> np.ndarray:
        """沿赛道随机起点 + 每回合独立随机数源（保证 (shape, seed) 可复现）。"""
        # 用 crc32 而不是内置 hash：后者受 PYTHONHASHSEED 影响，换个 shell 结果就变了
        self.rng = np.random.default_rng(zlib.crc32(f"{self.shape}|{seed}".encode()))
        n = len(self.path)
        self.idx0 = int(self.rng.integers(0, n))
        tangent = track.tangents(self.path)[self.idx0]
        lateral = float(self.rng.uniform(-0.25, 0.25))
        point = self.path[self.idx0] + track.normals(self.path)[self.idx0] * lateral
        yaw = float(np.arctan2(tangent[1], tangent[0]))

        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:3] = (point[0], point[1], yaw)
        self.data.qvel[:3] = 0.0
        mujoco.mj_forward(self.model, self.data)
        return self.pose()

    def pose(self) -> np.ndarray:
        return np.array([self.data.qpos[0], self.data.qpos[1], _wrap(self.data.qpos[2])])

    def velocity(self) -> tuple[float, float]:
        """世界系速度 → (前进速度, 角速度)。"""
        vx, vy, wz = self.data.qvel[:3]
        return float(np.hypot(vx, vy)), float(wz)

    # ---------------------------------------------------------------- 感知
    def raycast(self, pnt: np.ndarray, angles: np.ndarray, max_range: float = LIDAR_RANGE,
                z: float = 0.11) -> np.ndarray:
        out = np.empty(len(angles))
        for i, a in enumerate(angles):
            vec = np.array([np.cos(a), np.sin(a), 0.0])
            d = mujoco.mj_ray(self.model, self.data, np.array([pnt[0], pnt[1], z]), vec,
                              self._geomgroup, 1, self._bodyexclude, self._geomid)
            out[i] = max_range if d < 0 else min(float(d), max_range)
        return out

    def scan(self, pose: np.ndarray | None = None) -> np.ndarray:
        p = self.pose() if pose is None else pose
        ang = p[2] + self._beam_angles
        r = self.raycast(p, ang)
        r = r + self.rng.normal(0.0, self.sensor_noise, size=r.shape)
        return np.clip(r, 0.02, LIDAR_RANGE)

    def sectors(self, scan: np.ndarray) -> np.ndarray:
        """把 24 线雷达压成 24 维归一化特征（每束单独一维，前 12 束朝前）。"""
        return scan / LIDAR_RANGE

    # ---------------------------------------------------------------- 误差
    def errors(self, pose: np.ndarray | None = None) -> tuple[float, float]:
        """横向误差（左正）与航向误差（相对切线）。"""
        p = self.pose() if pose is None else pose
        i = self.nearest_index(p[:2])
        nrm = track.normals(self.path)[i]
        tan = track.tangents(self.path)[i]
        e = float(np.dot(p[:2] - self.path[i], nrm))
        psi = _wrap(p[2] - float(np.arctan2(tan[1], tan[0])))
        return e, psi

    def nearest_index(self, xy: np.ndarray, hint: int | None = None) -> int:
        n = len(self.path)
        if hint is None:
            d = np.linalg.norm(self.path - xy, axis=1)
            return int(np.argmin(d))
        window = 40
        idx = (hint + np.arange(-window, window + 1)) % n
        d = np.linalg.norm(self.path[idx] - xy, axis=1)
        return int(idx[int(np.argmin(d))])

    def progress(self) -> float:
        hint = getattr(self, "_hint", None)
        return float(self.nearest_index(self.pose()[:2], hint) / len(self.path))

    # ---------------------------------------------------------------- 执行
    def step(self, v: float, w: float) -> np.ndarray:
        """跑满一个 20 Hz 控制周期。"""
        return self.step_period(v, w, SUBSTEPS)

    def step_period(self, v: float, w: float, substeps: int) -> np.ndarray:
        """把 (v,w) 换成世界系速度指令，保持 substeps 个物理步（用于变控制频率）。"""
        yaw = self.pose()[2]
        self.data.ctrl[0] = v * np.cos(yaw)
        self.data.ctrl[1] = v * np.sin(yaw)
        self.data.ctrl[2] = w
        for _ in range(int(substeps)):
            mujoco.mj_step(self.model, self.data)
        self._hint = self.nearest_index(self.pose()[:2], getattr(self, "_hint", None))
        return self.pose()

    @property
    def beam_angles(self) -> np.ndarray:
        return self._beam_angles
