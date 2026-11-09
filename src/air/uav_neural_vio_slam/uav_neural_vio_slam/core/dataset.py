#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""VIO 数据集：飞一圈，把 (深度图, IMU 窗口, 相对位姿增量) 记下来。

采集用**真值**位姿算相对增量，因此没有闭环问题；训练时网络只吃深度与 IMU，
**看不到任何位姿**（``pose``/``yaw`` 只留给评测）。

一帧 = 10 Hz 的相机 + 该帧与上一帧之间的 10 个 IMU 采样（100 Hz）。
相对增量在**上一帧的机体系**里表示 —— 与网络输出的约定一致。
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .landmarks import DetectorConfig, LandmarkDetector, to_world_horizontal
from .world import LoopTrajectory, World, build_world

__all__ = ["VioEpisode", "VioDataset", "EpisodeSpec", "collect_episode",
           "collect_dataset", "depth_to_u8", "u8_to_depth", "DEPTH_MIN", "DEPTH_MAX"]

DEPTH_MIN, DEPTH_MAX = 0.2, 10.0
LIDAR_RANGE = 16.0          # 雷达整帧存成 uint8 时的量程
FRAME_DT = 0.1              # 10 Hz 关键帧
SIM_DT = 0.01               # 100 Hz 仿真
IMU_LEN = 10                # 每帧之间的 IMU 采样数
DEPTH_HW = (48, 64)


def _ranges_to_u8(r: np.ndarray, max_range: float = LIDAR_RANGE) -> np.ndarray:
    q = np.rint(np.asarray(r, np.float32) / max_range * 254.0) + 1.0
    q[np.asarray(r) <= 1e-6] = 0.0
    return np.clip(q, 0, 255).astype(np.uint8)


# --------------------------------------------------------------------------- #
def depth_to_u8(d: np.ndarray) -> np.ndarray:
    """深度 (H,W) 米 → uint8。0 表示无效像素（超量程或太近）。"""
    a = np.asarray(d, np.float32)
    q = np.rint((a - DEPTH_MIN) / (DEPTH_MAX - DEPTH_MIN) * 254.0) + 1.0
    # 超量程显式置 0：量化会把 20 m 夹到 255，解码成 10 m 的"有效回波"。
    q[(a <= 1e-6) | (a < DEPTH_MIN) | (a > DEPTH_MAX)] = 0.0
    return np.clip(q, 0, 255).astype(np.uint8)


def u8_to_depth(q: np.ndarray) -> np.ndarray:
    d = (np.asarray(q, np.float32) - 1.0) / 254.0 * (DEPTH_MAX - DEPTH_MIN) + DEPTH_MIN
    d[np.asarray(q) == 0] = 0.0
    return d


@dataclass
class VioEpisode:
    depth: np.ndarray          # (N, H, W) uint8
    imu: np.ndarray            # (N, K, 6) float32  [gyro(3), acc(3)]
    dp: np.ndarray             # (N, 3) float32 机体系位移增量
    dr: np.ndarray             # (N, 3) float32 机体系 rpy 增量
    pos: np.ndarray            # (N, 3) float32 真值世界位置
    yaw: np.ndarray            # (N,)  float32 真值偏航
    obs_r: np.ndarray          # (N, M) float32 路标测距
    obs_b: np.ndarray          # (N, M) float32 路标方位
    obs_n: np.ndarray          # (N,)  int32   有效观测个数
    lidar: np.ndarray = None   # (N, n_ray) uint8 整帧射线距离（建图用）
    meta: Dict[str, object] = field(default_factory=dict)

    def __len__(self) -> int:
        return int(len(self.depth))

    def ranges(self, i: int, lidar_detector) -> np.ndarray:
        """第 i 帧的射线距离（米），未命中为 0。"""
        if self.lidar is None:
            return np.zeros(lidar_detector.rays())
        q = self.lidar[i].astype(np.float32)
        r = (q - 1.0) / 254.0 * LIDAR_RANGE
        r[q == 0] = 0.0
        return r

    def subset(self, i0: int, i1: int) -> "VioEpisode":
        return VioEpisode(self.depth[i0:i1], self.imu[i0:i1], self.dp[i0:i1],
                          self.dr[i0:i1], self.pos[i0:i1], self.yaw[i0:i1],
                          self.obs_r[i0:i1], self.obs_b[i0:i1], self.obs_n[i0:i1],
                          None if self.lidar is None else self.lidar[i0:i1],
                          dict(self.meta))


@dataclass
class VioDataset:
    episodes: List[VioEpisode] = field(default_factory=list)
    landmarks: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))

    def __len__(self) -> int:
        return int(sum(len(e) for e in self.episodes))

    def stack(self, idx: Sequence[int] = None):
        """把若干 episode 的逐帧监督信号拼在一起，供训练用。"""
        eps = self.episodes if idx is None else [self.episodes[i] for i in idx]
        return (np.concatenate([e.depth for e in eps]),
                np.concatenate([e.imu for e in eps]),
                np.concatenate([e.dp for e in eps]),
                np.concatenate([e.dr for e in eps]))

    def save(self, path: str) -> None:
        d: Dict[str, np.ndarray] = {"landmarks": self.landmarks,
                                    "n_ep": np.asarray(len(self.episodes))}
        for i, e in enumerate(self.episodes):
            for k in ("depth", "imu", "dp", "dr", "pos", "yaw", "obs_r",
                      "obs_b", "obs_n", "lidar"):
                d["%s_%d" % (k, i)] = getattr(e, k)
        np.savez_compressed(path, **d)

    @classmethod
    def load(cls, path: str) -> "VioDataset":
        with np.load(path, allow_pickle=False) as d:
            n = int(d["n_ep"])
            eps = [VioEpisode(*[d["%s_%d" % (k, i)] for k in
                                ("depth", "imu", "dp", "dr", "pos", "yaw",
                                 "obs_r", "obs_b", "obs_n", "lidar")])
                   for i in range(n)]
            return cls(eps, np.array(d["landmarks"]))


# --------------------------------------------------------------------------- #
@dataclass
class EpisodeSpec:
    seed: int = 0
    loops: float = 2.0
    ax: float = 9.0
    ay: float = 6.5
    height: float = 1.7
    period: float = 40.0
    imu_seed: int = 0
    gyro_noise: float = 0.002
    acc_noise: float = 0.02

    def label(self) -> str:
        return "seed%d/loops%.1f/ax%.1f/ay%.1f/z%.2f" % (
            self.seed, self.loops, self.ax, self.ay, self.height)


def collect_episode(world: World, spec: EpisodeSpec,
                    camera: Optional[CameraConfig] = None,
                    detector: Optional[LandmarkDetector] = None,
                    max_obs: int = 12, verbose: bool = False) -> VioEpisode:
    """按 ``spec`` 飞一圈并录一集。

    ``VioDataset`` 的读取与评估不依赖 mujoco，仿真相关模块在此延迟导入。
    """
    from uav_mujoco_sim.core import (GeometricController, Quadrotor, quat_to_mat,
                                     quat_to_euler)
    from uav_mujoco_sim.core.sensors import (CameraConfig, ImuSensor, LidarSensor,
                                             LidarConfig, VisionSensors)

    det = detector or LandmarkDetector()
    cam = camera or CameraConfig(width=DEPTH_HW[1], height=DEPTH_HW[0],
                                 quality="fast", depth_max_range=DEPTH_MAX)
    traj = LoopTrajectory(ax=spec.ax, ay=spec.ay, height=spec.height,
                          period=spec.period)

    quad = Quadrotor(scene=world.scene, dt=SIM_DT, substeps=4)
    ctrl = GeometricController(quad.params)
    vision = VisionSensors(quad.model, cam)
    imu = ImuSensor(quad.model, gyro_noise=spec.gyro_noise,
                    acc_noise=spec.acc_noise, gyro_bias=[2e-3, -1e-3, 1.5e-3],
                    acc_bias=[0.02, -0.015, 0.03], rate_hz=100.0,
                    seed=spec.imu_seed)
    lid = LidarSensor(quad.model, LidarConfig(
        n_azimuth=det.n_az, elevations_deg=list(det.cfg.elevations_deg),
        max_range=det.cfg.max_range, min_range=det.cfg.min_range, frame="body"))

    n_steps = int(round(traj.period * spec.loops / SIM_DT))
    per = int(round(FRAME_DT / SIM_DT))
    assert per == IMU_LEN

    p0, _, _, yaw0, _ = traj.eval(0.0)
    quad.reset(pos=tuple(p0), yaw=float(yaw0))
    imu.reset()

    rec = {k: [] for k in ("depth", "imu", "dp", "dr", "pos", "yaw",
                           "obs_r", "obs_b", "obs_n", "lidar")}
    win: List[np.ndarray] = []
    prev_p = quad.state().pos.copy()
    prev_R = quat_to_mat(quad.state().quat)
    prev_rpy = np.asarray(quat_to_euler(quad.state().quat))

    try:
        for k in range(n_steps):
            t = k * SIM_DT
            pd, vd, ad, yd, _ = traj.eval(t)
            quad.step_with_control(ctrl.act(quad, pd, vd, ad, yd))
            r = imu.read(quad.data, quad.time)
            win.append(np.concatenate([r.gyro, r.acc]))

            if (k + 1) % per == 0:
                st = quad.state()
                R = quat_to_mat(st.quat)
                rpy = np.asarray(quat_to_euler(st.quat))
                dep = vision.depth(quad.data)
                pts = lid.scan(quad.data)
                rng_img = det.ranges_from_points(pts)
                obs = det.detect(rng_img)
                # 用完整的姿态把观测转到世界水平系：只补偏航的话，
                # 俯仰/横滚会把 ±15° 那两层的方位掰偏最多 7°
                obs = to_world_horizontal(obs, R, float(st.yaw), det.cfg.max_range)
                rr, bb, nn = _pack_obs(obs, max_obs)
                rec["depth"].append(depth_to_u8(dep))
                rec["lidar"].append(_ranges_to_u8(rng_img))
                rec["imu"].append(np.asarray(win, np.float32))
                rec["dp"].append((prev_R.T @ (st.pos - prev_p)).astype(np.float32))
                d = rpy - prev_rpy
                rec["dr"].append((np.arctan2(np.sin(d), np.cos(d))).astype(np.float32))
                rec["pos"].append(st.pos.astype(np.float32))
                rec["yaw"].append(float(st.yaw))
                rec["obs_r"].append(rr)
                rec["obs_b"].append(bb)
                rec["obs_n"].append(nn)
                win.clear()
                prev_p, prev_R, prev_rpy = st.pos.copy(), R, rpy
    finally:
        vision.close()

    ep = VioEpisode(
        depth=np.asarray(rec["depth"]), imu=np.asarray(rec["imu"]),
        dp=np.asarray(rec["dp"]), dr=np.asarray(rec["dr"]),
        pos=np.asarray(rec["pos"]), yaw=np.asarray(rec["yaw"], np.float32),
        obs_r=np.asarray(rec["obs_r"], np.float32),
        obs_b=np.asarray(rec["obs_b"], np.float32),
        obs_n=np.asarray(rec["obs_n"], np.int32),
        lidar=np.asarray(rec["lidar"]),
        meta={"spec": spec.label(), "period": traj.period, "world": world.name})
    if verbose:
        print("  %-34s %4d 帧  |Δp|均值 %.4f m  观测 %.1f 个/帧"
              % (spec.label(), len(ep), float(np.linalg.norm(ep.dp, axis=1).mean()),
                 float(ep.obs_n.mean())))
    return ep


def _pack_obs(obs, max_obs: int) -> Tuple[np.ndarray, np.ndarray, int]:
    n = min(len(obs), max_obs)
    rr = np.zeros(max_obs, np.float32)
    bb = np.zeros(max_obs, np.float32)
    for i in range(n):
        rr[i], bb[i] = obs[i].rng, obs[i].bear
    return rr, bb, n


DEFAULT_SPECS = [
    EpisodeSpec(0, loops=2.0, ax=9.0, ay=6.5, height=1.70, period=40.0, imu_seed=0),
    EpisodeSpec(0, loops=2.0, ax=9.0, ay=6.5, height=1.70, period=40.0, imu_seed=1),
    EpisodeSpec(0, loops=1.5, ax=7.0, ay=5.0, height=1.55, period=33.0, imu_seed=2),
    EpisodeSpec(0, loops=2.0, ax=10.5, ay=7.5, height=1.85, period=46.0, imu_seed=3),
    EpisodeSpec(0, loops=1.0, ax=11.0, ay=8.0, height=1.95, period=52.0, imu_seed=4),
]


def collect_dataset(specs: Sequence[EpisodeSpec] = None, seed: int = 0,
                    verbose: bool = True) -> VioDataset:
    world = build_world(seed=seed)
    eps = []
    for s in (specs if specs is not None else DEFAULT_SPECS):
        eps.append(collect_episode(world, s, verbose=verbose))
    return VioDataset(eps, world.positions.astype(np.float32))
