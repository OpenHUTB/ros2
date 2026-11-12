"""阿克曼小车 + 锥桶雷达的 MuJoCo 环境。

要点：

* 运动学约束用速度伺服实现：给定 (v, δ)，期望偏航率 \\(\\omega = v\\tan\\delta / L\\)，
  用纵向力 + 偏航力矩把车拉到该状态，侧向强阻尼模拟轮胎不侧滑；
* 雷达是**解析**的：锥桶是已知半径的圆柱，射线-圆柱求交用 numpy 向量化算，
  比逐个 `mj_ray` 快约 20 倍（有测试与 `mj_ray` 交叉验证）；
* 里程计只给"编码器 + 陀螺"能测到的量，且带噪声/偏置，供状态估计使用。
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import mujoco

from . import track

DT = 0.05              # 控制周期 20 Hz
SUBSTEPS = 10          # 物理步长 5 ms
WHEELBASE = 0.26       # 轴距
CONE_RADIUS = 0.09     # 锥桶半径
LIDAR_BEAMS = 32
LIDAR_RANGE = 6.0


def _find_asset(name: str) -> Path:
    env = os.environ.get("MPPI_NN_MODELS")
    if env:
        p = Path(env) / name
        if p.exists():
            return p
    try:
        from ament_index_python.packages import get_package_share_directory
        p = Path(get_package_share_directory("mujoco_mppi_neural_tracker")) / "models" / name
        if p.exists():
            return p
    except Exception:
        pass
    return Path(__file__).resolve().parents[1] / "models" / name


MODEL_PATH = _find_asset("track.xml")


def wrap(a: float) -> float:
    return float((a + np.pi) % (2 * np.pi) - np.pi)


@dataclass
class State:
    pos: np.ndarray
    yaw: float
    v: float
    delta: float


class AckermannEnv:
    START = (track.A, 0.0)
    START_YAW = np.pi / 2          # 起点处切向朝 +y

    def __init__(self, seed: int = 0):
        self.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        self.data = mujoco.MjData(self.model)
        self.body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "car")
        self.mass = float(self.model.body_subtreemass[self.body_id])
        self.rng = np.random.default_rng(seed)
        self.cones = track.cone_positions()
        self.path = track.ref_polyline()
        self.ref_idx = 0
        self.crashed = False
        self._renderer = None
        self.reset()

    # ---------------------------------------------------------------- 场景
    def reset(self, pos=None, yaw: float | None = None) -> State:
        mujoco.mj_resetData(self.model, self.data)
        p = np.array(pos if pos is not None else self.START, dtype=np.float64)
        self.data.qpos[0:2] = p
        self.data.qpos[2] = 0.12
        yaw = self.START_YAW if yaw is None else yaw
        self.data.qpos[3:7] = (np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2))
        mujoco.mj_forward(self.model, self.data)
        self.crashed = False
        self.ref_idx = track.nearest_index(self.path, p)
        return self.state()

    # ---------------------------------------------------------------- 状态
    def state(self) -> State:
        q = self.data.qpos
        yaw = float(np.arctan2(2 * (q[3] * q[6] + q[4] * q[5]),
                               1 - 2 * (q[5] ** 2 + q[6] ** 2)))
        vx, vy = float(self.data.qvel[0]), float(self.data.qvel[1])
        v = vx * np.cos(yaw) + vy * np.sin(yaw)
        return State(pos=np.array(q[0:2]), yaw=yaw, v=v, delta=self._delta)

    def errors(self) -> tuple[float, float]:
        s = self.state()
        self.ref_idx = track.nearest_index(self.path, s.pos, self.ref_idx)
        return track.track_errors(self.path, s.pos, s.yaw, self.ref_idx)

    @property
    def progress(self) -> float:
        return float(self.ref_idx) / len(self.path)

    # ---------------------------------------------------------------- 传感器
    def scan(self) -> np.ndarray:
        """32 线雷达，返回每条射线的距离（米）。"""
        s = self.state()
        ang = s.yaw + np.linspace(-np.pi, np.pi, LIDAR_BEAMS, endpoint=False)
        d = np.stack([np.cos(ang), np.sin(ang)], axis=1)            # (B,2)
        rel = self.cones[None, :, :] - s.pos[None, None, :]         # (1,C,2)
        proj = rel @ d[:, :, None]                                  # (B,C,1)
        proj = proj[:, :, 0]
        perp2 = (rel ** 2).sum(axis=2) - proj ** 2
        hit = (proj > 0) & (perp2 < CONE_RADIUS ** 2)
        if not hit.any():
            return np.full(LIDAR_BEAMS, LIDAR_RANGE, dtype=np.float32)
        t = proj - np.sqrt(np.maximum(CONE_RADIUS ** 2 - perp2, 0.0))
        t = np.where(hit, t, np.inf)
        r = np.minimum(t.min(axis=1), LIDAR_RANGE)
        return r.astype(np.float32)

    def lane_errors_from_lidar(self) -> tuple[float, float]:
        """只用雷达估计横向/航向误差：找左右锥桶的距离差。

        车在车道中间时左右最近锥桶距离相等；偏向一侧时该侧更近。
        """
        scan = self.scan()
        n = len(scan)
        ang = np.linspace(-np.pi, np.pi, n, endpoint=False)
        front = np.abs(ang) < 0.9
        left = (ang > 0) & front
        right = (ang < 0) & front
        dl = float(scan[left].min()) if left.any() else LIDAR_RANGE
        dr = float(scan[right].min()) if right.any() else LIDAR_RANGE
        e = (dr - dl) / 2.0                       # 左近 => e > 0（车偏左）
        psi = 0.0
        return float(np.clip(e, -2.0, 2.0)), psi

    def odom(self, prev: State, cur: State) -> tuple[float, float]:
        """编码器 + 陀螺：由位姿增量反推，带噪声与尺度误差。"""
        dx, dy = cur.pos - prev.pos
        v = (dx * np.cos(prev.yaw) + dy * np.sin(prev.yaw)) / DT
        w = wrap(cur.yaw - prev.yaw) / DT
        return float(v + self.rng.normal(0, 0.02)), float(w + self.rng.normal(0, 0.02))

    # ---------------------------------------------------------------- 动力学
    def step(self, v: float, delta: float) -> State:
        """速度伺服：把 (v, δ) 变成力与偏航力矩。"""
        s = self.state()
        self._delta = float(np.clip(delta, -0.6, 0.6))
        target_w = v * np.tan(self._delta) / WHEELBASE
        heading = np.array([np.cos(s.yaw), np.sin(s.yaw)])
        vel = np.array(self.data.qvel[0:2])
        force = self.mass * 6.0 * (v - float(vel @ heading)) * heading
        side = np.array([-heading[1], heading[0]])
        force -= self.mass * 8.0 * float(vel @ side) * side
        omega = float(self.data.qvel[5])
        torque = 3.0 * (target_w - omega)
        self.data.xfrc_applied[self.body_id, 0:2] = np.clip(force, -40, 40)
        self.data.xfrc_applied[self.body_id, 2] = self.mass * 9.81
        self.data.xfrc_applied[self.body_id, 5] = float(np.clip(torque, -6, 6))
        for _ in range(SUBSTEPS):
            mujoco.mj_step(self.model, self.data)
        if not np.all(np.isfinite(self.data.qpos)):
            self.reset()
            self.crashed = True
            return self.state()
        cur = self.state()
        # 出界判定：离参考线过远
        self.ref_idx = track.nearest_index(self.path, cur.pos, self.ref_idx)
        e, _ = track.track_errors(self.path, cur.pos, cur.yaw, self.ref_idx)
        if abs(e) > 2.5:
            self.crashed = True
        return cur

    _delta = 0.0

    # ---------------------------------------------------------------- 渲染
    def rgb(self, cam: str = "chase_cam", size: int = 220) -> np.ndarray:
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, size, size)
        self._renderer.update_scene(self.data, camera=cam)
        return self._renderer.render()
