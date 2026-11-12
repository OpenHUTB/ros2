"""闭环：传感器估计 -> 三个控制器（MPPI / 蒸馏网络 / 纯追踪）-> 指标与出图。"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np

from . import track
from .env import DT, AckermannEnv, wrap
from .mppi import MPPIConfig, MPPIController, pure_pursuit
from .net import DELTA_MAX, V_MAX, V_MIN, TrackerNet, build_features


@dataclass
class EpisodeResult:
    controller: str
    noise: float
    seed: int
    rms: float
    max_err: float
    completed: bool
    steps: int
    mean_ms: float
    trace: list = field(default_factory=list)
    frames: list = field(default_factory=list)


class PoseEstimator:
    """里程计航迹推算 + 雷达横向误差修正。

    纯航迹推算会随转弯慢慢漂；雷达测到的左右锥桶距离给出**横向**的绝对观测，
    按增益 α 把估计位姿沿参考线法向拉回去。这是只有一维观测的轻量融合，
    用来保证控制器吃的是"传感器估计值"而不是真值。
    """

    def __init__(self, env: AckermannEnv, alpha: float = 0.45):
        s = env.state()
        self.pos = s.pos.copy()
        self.yaw = s.yaw
        self.alpha = alpha
        self.idx = track.nearest_index(env.path, self.pos)

    def predict(self, v: float, w: float, dt: float = DT) -> None:
        self.pos[0] += v * np.cos(self.yaw) * dt
        self.pos[1] += v * np.sin(self.yaw) * dt
        self.yaw = wrap(self.yaw + w * dt)

    def correct(self, path: np.ndarray, e_meas: float) -> None:
        self.idx = track.nearest_index(path, self.pos, self.idx)
        e_est, _ = track.track_errors(path, self.pos, self.yaw, self.idx)
        p = path[self.idx]
        nxt = path[(self.idx + 1) % len(path)]
        t = nxt - p
        t = t / (np.linalg.norm(t) + 1e-12)
        nrm = np.array([-t[1], t[0]])
        self.pos = self.pos + nrm * self.alpha * (e_meas - e_est)

    @property
    def pose(self):
        return np.array([self.pos[0], self.pos[1], self.yaw])


def run_episode(env: AckermannEnv, controller: str = "mppi", noise: float = 0.0,
                seed: int = 0, steps: int = 1200, record: bool = False,
                net: TrackerNet | None = None, path=None) -> EpisodeResult:
    path = env.path if path is None else path
    env.reset()
    env.rng = np.random.default_rng(1000 + seed)
    est = PoseEstimator(env)
    mppi = MPPIController(path, MPPIConfig(), seed=seed) if controller == "mppi" else None

    delta_prev, v_prev = 0.0, 0.9
    errs, frames, trace, ms = [], [], [], []
    state_prev = env.state()
    step = 0

    for step in range(steps):
        # ---- 感知：雷达 -> 横向误差（可加测量噪声），并修正估计位姿
        e_true, psi_true = env.errors()
        e_meas = e_true + env.rng.normal(0.0, 0.04 + 0.10 * noise)
        est.correct(path, e_meas)
        pose = est.pose
        idx = track.nearest_index(path, pose[:2], est.idx)
        e_est, psi_est = track.track_errors(path, pose[:2], pose[2], idx)

        # ---- 控制
        t0 = time.perf_counter()
        if controller == "mppi":
            v_cmd, delta = mppi.act(pose, v_now=state_prev.v)
        elif controller == "nn":
            feats = build_features(e_est, psi_est, state_prev.v, delta_prev, v_prev,
                                   path, idx, pose)
            v_cmd, delta = net.act(feats)
        else:
            v_cmd, delta = pure_pursuit(pose, path, idx)
        ms.append((time.perf_counter() - t0) * 1000.0)

        if record and step % 4 == 0:
            frames.append((env.rgb("top_cam"), np.array([e_true, psi_true, v_cmd, delta])))

        new_state = env.step(v_cmd, delta)
        v_o, w_o = env.odom(state_prev, new_state)     # 编码器 + 陀螺（带噪声）
        est.predict(v_o, w_o)
        state_prev = new_state

        delta_prev, v_prev = delta, v_cmd
        errs.append(abs(e_true))
        trace.append((new_state.pos.copy(), pose[:2].copy(), e_true))

        if env.crashed or env.progress > 0.98:
            break

    ms_arr = np.array(ms)
    return EpisodeResult(
        controller=controller, noise=noise, seed=seed,
        rms=round(float(np.sqrt(np.mean(np.square(errs)))), 3),
        max_err=round(float(np.max(errs)), 3),
        completed=bool(env.progress > 0.98 and not env.crashed),
        steps=step + 1, mean_ms=round(float(ms_arr.mean()), 2),
        trace=trace, frames=frames,
    )


def collect_expert(env: AckermannEnv, episodes: int = 4, seed: int = 0,
                   steps: int = 1200, log=print):
    """用 MPPI 跑几圈，记录 (特征, 动作)，供神经网络模仿。"""
    path = env.path
    xs, ys = [], []
    for ep in range(episodes):
        env.reset()
        env.rng = np.random.default_rng(2000 + seed + ep)
        est = PoseEstimator(env)
        mppi = MPPIController(path, MPPIConfig(), seed=seed + ep)
        delta_prev, v_prev = 0.0, 0.9
        for _ in range(steps):
            e_meas, _ = env.lane_errors_from_lidar()
            est.correct(path, e_meas)
            pose = est.pose
            idx = track.nearest_index(path, pose[:2], est.idx)
            e_est, psi_est = track.track_errors(path, pose[:2], pose[2], idx)
            v_now = env.state().v
            v_cmd, delta = mppi.act(pose, v_now=v_now)
            xs.append(build_features(e_est, psi_est, v_now, delta_prev, v_prev,
                                     path, idx, pose))
            ys.append([np.clip(delta / DELTA_MAX, -1, 1),
                       np.clip((v_cmd - V_MIN) / (V_MAX - V_MIN), 0, 1)])
            state_prev = env.state()
            delta_prev, v_prev = delta, v_cmd
            new_state = env.step(v_cmd, delta)
            v_o, w_o = env.odom(state_prev, new_state)
            est.predict(v_o, w_o)
            if env.crashed or env.progress > 0.98:
                break
        log(f"专家回合 {ep + 1}/{episodes}: 进度 {env.progress:.2f}，样本累计 {len(xs)}")
    return np.stack(xs).astype(np.float32), np.stack(ys).astype(np.float32)
