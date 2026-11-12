"""传感器仿真：2D 激光雷达、深度相机、IMU、里程计。

全部基于 MuJoCo 的射线求交（``mj_ray``），不依赖 OpenGL，因此在没有显卡的
虚拟机 / 服务器上也能产出完全一致的感知数据——这一点对课程作业的可复现性很关键。
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

import mujoco


def _quat_to_rpy(q) -> tuple:
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class Lidar:
    """360° 单线激光雷达（水平面扫描）。

    射线只与 group 0~2 的几何体求交；机器人自身几何被分组到 3，因此天然被忽略。
    """

    def __init__(self, model: mujoco.MjModel, n_beams: int = 180,
                 max_range: float = 8.0, site: str = "lidar", noise: float = 0.0):
        self.model = model
        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site)
        self.n = int(n_beams)
        self.max_range = float(max_range)
        self.noise = float(noise)
        self.angles = np.linspace(-math.pi, math.pi, self.n, endpoint=False)
        self.geomgroup = np.array([1, 1, 1, 0, 0, 0], dtype=np.uint8)
        self._geomid = np.zeros(1, dtype=np.int32)

    def scan(self, data: mujoco.MjData, rng: Optional[np.random.Generator] = None) -> np.ndarray:
        p = data.site_xpos[self.site_id]
        r = data.site_xmat[self.site_id].reshape(3, 3)
        # 雷达坐标系 x 轴为 0°，绕世界 z 轴旋转
        fwd = r @ np.array([1.0, 0.0, 0.0])
        yaw = math.atan2(fwd[1], fwd[0])
        out = np.empty(self.n, dtype=np.float64)
        geomid = np.zeros(1, dtype=np.int32)
        for i, a in enumerate(self.angles):
            ang = yaw + a
            vec = np.array([math.cos(ang), math.sin(ang), 0.0])
            dist = mujoco.mj_ray(self.model, data, np.asarray(p, dtype=np.float64), vec,
                                 self.geomgroup, 1, -1, geomid)
            out[i] = dist if dist >= 0 else self.max_range
        if self.noise > 0 and rng is not None:
            out = out + rng.normal(0.0, self.noise, size=self.n)
        return np.clip(out, 0.05, self.max_range)

    def hit_points(self, data, ranges: np.ndarray) -> np.ndarray:
        """把扫描转换成世界坐标系下的点云 (N,2)。"""
        p = data.site_xpos[self.site_id]
        r = data.site_xmat[self.site_id].reshape(3, 3)
        fwd = r @ np.array([1.0, 0.0, 0.0])
        yaw = math.atan2(fwd[1], fwd[0])
        a = yaw + self.angles
        return np.stack([p[0] + ranges * np.cos(a), p[1] + ranges * np.sin(a)], axis=1)


class DepthCamera:
    """针孔深度相机：对每个像素发一条射线，返回米制深度。

    分辨率刻意取小（默认 40×30），既够端到端网络使用，又能保证 CPU 上实时。
    """

    def __init__(self, model: mujoco.MjModel, width: int = 40, height: int = 30,
                 fovy_deg: float = 70.0, max_range: float = 12.0, camera: str = "fpv"):
        self.model = model
        self.width = int(width)
        self.height = int(height)
        self.max_range = float(max_range)
        self.cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
        self.geomgroup = np.array([1, 1, 1, 0, 0, 0], dtype=np.uint8)
        fy = 0.5 * height / math.tan(math.radians(fovy_deg) / 2.0)
        fx = fy
        ys, xs = np.mgrid[0:height, 0:width]
        self._dir_cam = np.stack([
            (xs - width / 2.0 + 0.5) / fx,
            -(ys - height / 2.0 + 0.5) / fy,
            -np.ones_like(xs, dtype=np.float64),
        ], axis=-1).reshape(-1, 3)
        self._dir_cam /= np.linalg.norm(self._dir_cam, axis=1, keepdims=True)
        self._geomid = np.zeros(1, dtype=np.int32)

    def depth(self, data: mujoco.MjData) -> np.ndarray:
        pos = data.cam_xpos[self.cam_id]
        mat = data.cam_xmat[self.cam_id].reshape(3, 3)
        dirs = self._dir_cam @ mat.T
        out = np.empty(len(dirs), dtype=np.float64)
        geomid = np.zeros(1, dtype=np.int32)
        for i in range(len(dirs)):
            d = mujoco.mj_ray(self.model, data, np.asarray(pos, dtype=np.float64),
                              dirs[i].astype(np.float64), self.geomgroup, 1, -1, geomid)
            out[i] = d if d >= 0 else self.max_range
        return np.clip(out, 0.1, self.max_range).reshape(self.height, self.width)


class IMU:
    """由真值位姿差分得到的 IMU（含噪声），用于姿态反馈。"""

    def __init__(self, noise: float = 0.002, gyro_noise: float = 0.01):
        self.noise = noise
        self.gyro_noise = gyro_noise
        self._prev = None

    def read(self, base_quat, base_vel, rng: Optional[np.random.Generator] = None):
        roll, pitch, yaw = _quat_to_rpy(base_quat)
        gyro = np.zeros(3)
        if self._prev is not None:
            prev_roll, prev_pitch, prev_yaw = self._prev
            gyro = np.array([roll - prev_roll, pitch - prev_pitch,
                             math.atan2(math.sin(yaw - prev_yaw), math.cos(yaw - prev_yaw))]) / 0.01
        self._prev = (roll, pitch, yaw)
        if rng is not None:
            roll += rng.normal(0, self.noise)
            pitch += rng.normal(0, self.noise)
            yaw += rng.normal(0, self.noise)
            gyro += rng.normal(0, self.gyro_noise, 3)
        return {"roll": roll, "pitch": pitch, "yaw": yaw, "gyro": gyro,
                "accel": np.asarray(base_vel, dtype=float)}


class Odometry:
    """腿式里程计近似：对实测速度积分，并叠加随时间增长的角度漂移。"""

    def __init__(self, drift_per_m: float = 0.02, noise: float = 0.01):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.drift_per_m = drift_per_m
        self.noise = noise

    def reset(self, x=0.0, y=0.0, yaw=0.0):
        self.x, self.y, self.yaw = x, y, yaw

    def update(self, vx: float, vy: float, wz: float, dt: float,
               rng: Optional[np.random.Generator] = None) -> dict:
        yaw = self.yaw
        self.x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        self.y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        dist = math.hypot(vx, vy) * dt
        self.yaw += wz * dt + self.drift_per_m * dist
        if rng is not None:
            self.x += rng.normal(0, self.noise * dt)
            self.y += rng.normal(0, self.noise * dt)
        return {"x": self.x, "y": self.y, "yaw": self.yaw}
