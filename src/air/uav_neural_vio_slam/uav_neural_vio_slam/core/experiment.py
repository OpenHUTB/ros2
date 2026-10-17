#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对比实验：同一前端、同一数据集、只换后端。

三个后端：

    ekf          EKF-SLAM，固定观测噪声
    ekf_learned  EKF-SLAM，观测噪声由网络按观测几何预测（本模块提出）
    fastslam     FastSLAM 1.0（粒子滤波轨迹 + 逐粒子路标 EKF）

外加一条"GPS 中断"曲线：GPS 每 1 s 给一次绝对位置（σ=0.5 m），
在指定时间窗里切断，看三个后端各自能撑多久、恢复得多快。
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .dataset import VioDataset, VioEpisode
from .ekf_slam import EkfConfig, EkfSlam, wrap
from .fastslam import FastSlam, FastSlamConfig
from .landmarks import LandmarkDetector
from .mapping import GridConfig, OccupancyGrid
from .metrics import (apply_se2, ate, landmark_errors, landmark_shape_rmse,
                      local_transform, map_iou, poses_local, rpe)

__all__ = ["BackendConfig", "RunResult", "BACKENDS", "run_backend",
           "run_grid", "summarize", "table"]

BACKENDS = ("odom", "ekf", "ekf_learned", "fastslam")
BACKEND_LABEL = {"odom": "纯里程计（前端）", "ekf": "EKF-SLAM",
                 "ekf_learned": "EKF-SLAM + 神经观测模型",
                 "fastslam": "FastSLAM 1.0"}


class DeadReckoning:
    """只积分前端增量、不做任何修正。作为"没有 SLAM"的基准线。"""

    def __init__(self, *a, **kw):
        self.p = np.zeros(3)
        self.traj: List[np.ndarray] = []

    def predict(self, dp_b, dyaw, sigma=None):
        p = self.p
        c, s = math.cos(p[2]), math.sin(p[2])
        self.p = np.array([p[0] + c * dp_b[0] - s * dp_b[1],
                           p[1] + s * dp_b[0] + c * dp_b[1],
                           float(wrap(p[2] + dyaw))])

    def update_landmarks(self, *a):
        return 0

    def update_gps(self, *a):
        pass

    def record(self):
        self.traj.append(self.p.copy())

    def trajectory(self):
        return np.asarray(self.traj)

    @property
    def landmarks(self):
        return np.zeros((0, 2))


@dataclass
class BackendConfig:
    gps_period: float = 1.0
    gps_sigma: float = 0.5
    gps_start: float = 1.0
    outage: Tuple[float, float] = (0.0, 0.0)     # (起, 止) 秒；起=止 表示不中断
    seed: int = 0
    build_map: bool = True
    frame_dt: float = 0.1


@dataclass
class RunResult:
    backend: str
    episode: int
    outage: Tuple[float, float]
    metrics: Dict[str, float] = field(default_factory=dict)
    poses: Optional[np.ndarray] = None
    gt: Optional[np.ndarray] = None
    landmarks: Optional[np.ndarray] = None
    gt_landmarks: Optional[np.ndarray] = None
    grid: Optional[np.ndarray] = None
    gt_grid: Optional[np.ndarray] = None

    def key(self) -> str:
        return "%s|ep%d|out%.0f-%.0f" % (self.backend, self.episode,
                                         self.outage[0], self.outage[1])


# --------------------------------------------------------------------------- #
def _poses3(ep: VioEpisode) -> np.ndarray:
    p = np.asarray(ep.pos, float)[:, :2]
    yaw = np.asarray(ep.yaw, float)
    return np.column_stack([p, yaw])


def _gps_series(gt: np.ndarray, cfg: BackendConfig, n: int, rng) -> np.ndarray:
    """返回 (n,3)：[t 是否可用, x, y]。"""
    t = np.arange(n) * cfg.frame_dt
    usable = (t >= cfg.gps_start)
    if cfg.outage[1] > cfg.outage[0]:
        usable &= ~((t >= cfg.outage[0]) & (t < cfg.outage[1]))
    # 每 gps_period 秒来一次（相位固定，保证各后端看到的观测完全一样）
    tick = np.zeros(n, dtype=bool)
    k = int(round(cfg.gps_start / cfg.frame_dt))
    step = int(round(cfg.gps_period / cfg.frame_dt))
    idx = np.arange(k, n, max(step, 1))
    tick[idx] = True
    usable &= tick
    noise = rng.normal(0.0, cfg.gps_sigma, (n, 2))
    return usable, gt[:, :2] + noise


def _rays2d(det: LandmarkDetector):
    """把三维射线投影到水平面：返回 (单位方向 (R,2), 水平/三维长度比 (R,))。"""
    d2 = det.dirs[:, :2]
    n = np.linalg.norm(d2, axis=1)
    return d2 / np.maximum(n, 1e-9)[:, None], n


def run_backend(ep: VioEpisode, pred: Dict[str, np.ndarray], backend: str,
                cfg: Optional[BackendConfig] = None, obs_model=None,
                det: Optional[LandmarkDetector] = None,
                make_map: bool = True) -> RunResult:
    cfg = cfg or BackendConfig()
    det = det or LandmarkDetector()
    n = len(ep)
    gt_world = _poses3(ep)
    gt = poses_local(gt_world)                 # 换到第一帧坐标系，与估计器同规范
    rng = np.random.default_rng(cfg.seed)

    ecfg = EkfConfig(gps_sigma=cfg.gps_sigma)
    if backend == "odom":
        slam = DeadReckoning()
    elif backend == "ekf":
        slam = EkfSlam(ecfg, obs_model=None)
    elif backend == "ekf_learned":
        slam = EkfSlam(ecfg, obs_model=obs_model)
    elif backend == "fastslam":
        slam = FastSlam(FastSlamConfig(seed=cfg.seed, gps_sigma=cfg.gps_sigma),
                        obs_model=obs_model)
    else:
        raise KeyError("未知后端 %r，可选 %s" % (backend, list(BACKENDS)))

    usable, gps = _gps_series(gt, cfg, n, rng)
    sigma = pred["sigma"]
    t0 = time.time()
    for t in range(n):
        slam.predict(pred["dp"][t], float(pred["dr"][t, 2]), sigma[t])
        slam.update_landmarks(ep.obs_r[t], ep.obs_b[t], int(ep.obs_n[t]))
        if usable[t]:
            slam.update_gps(gps[t])
        slam.record()
    wall = time.time() - t0

    est = slam.trajectory()
    m = {}
    m.update(ate(est, gt))
    m.update(rpe(est, gt, delta=10))
    m["wall_s"] = wall
    m["n_landmarks"] = float(len(slam.landmarks))
    if len(ep.obs_n):
        m["obs_per_frame"] = float(np.mean(ep.obs_n))

    # 真值路标与 gt 换到同一首帧坐标系：首帧 yaw=0.965 rad，10 m 处错开 9 m，
    # 不换坐标系的最近邻配对退化为随机（实测 rmse 0.71 m）。
    gt_lm_world = np.asarray(getattr(ep, "_gt_landmarks", []), float).reshape(-1, 2)
    gt_lm = apply_se2(local_transform(gt_world), gt_lm_world) \
        if len(gt_lm_world) else gt_lm_world

    res = RunResult(backend, getattr(ep, "_index", -1), cfg.outage, m,
                    poses=est, gt=gt, landmarks=slam.landmarks,
                    gt_landmarks=gt_lm)

    if make_map and ep.lidar is not None:
        u2, scale = _rays2d(det)
        gc = GridConfig()
        g_est, g_gt = OccupancyGrid(gc), OccupancyGrid(gc)
        for t in range(n):
            r2 = ep.ranges(t, det) * scale
            g_est.integrate(est[t], r2, u2, det.cfg.max_range)
            g_gt.integrate(gt[t], r2, u2, det.cfg.max_range)
        res.grid = g_est.probability()
        res.gt_grid = g_gt.probability()
        res.metrics["map_iou"] = map_iou(res.grid, res.gt_grid)
        # 0.10 m 分辨率上墙只有一格宽，位姿差 0.4 m 就整体错开 —— 原生分辨率
        # 的 IoU 量的是位姿误差而不是地图质量。同时给出 0.5 m 降采样版本。
        res.metrics["map_iou_coarse"] = map_iou(_coarse(res.grid, 5),
                                                _coarse(res.gt_grid, 5))
    return res


def _coarse(g: np.ndarray, k: int) -> np.ndarray:
    """栅格降采样：每 k×k 块取最大值（占据优先）。"""
    n = g.shape[0] // k * k
    return g[:n, :n].reshape(n // k, k, n // k, k).max(axis=(1, 3))


# --------------------------------------------------------------------------- #
def run_grid(ds: VioDataset, frontend, backends: Sequence[str] = BACKENDS,
             outages: Sequence[Tuple[float, float]] = ((0.0, 0.0),),
             episodes: Sequence[int] = (0, 1, 2, 3, 4),
             obs_model=None, verbose: bool = True, make_map: bool = True
             ) -> List[RunResult]:
    """``frontend`` 可以是 net（会被调 predict_episode）或现成的预测字典。"""
    from .training import predict_episode
    det = LandmarkDetector()
    results: List[RunResult] = []
    preds: Dict[int, Dict[str, np.ndarray]] = {}
    for i in episodes:
        ep = ds.episodes[i]
        preds[i] = frontend if isinstance(frontend, dict) else predict_episode(frontend, ep)
    total = len(episodes) * len(backends) * len(outages)
    k = 0
    for i in episodes:
        ep = ds.episodes[i]
        setattr(ep, "_index", i)
        setattr(ep, "_gt_landmarks", ds.landmarks)
        for out in outages:
            for b in backends:
                k += 1
                cfg = BackendConfig(outage=out, seed=1000 + i)
                r = run_backend(ep, preds[i], b, cfg, obs_model=obs_model,
                                det=det, make_map=make_map)
                results.append(r)
                lm = _lm(r)       # 路标指标与是否打印无关
                if verbose:
                    print("  [%2d/%2d] ep%d %-12s outage=%2.0f-%2.0fs  "
                          "ATE=%.3f m  RPE=%.3f m  路标=%.3f m  IoU=%.2f  (%.1fs)"
                          % (k, total, i, b, out[0], out[1],
                             r.metrics["ate_raw"], r.metrics["rpe_t"],
                             lm, r.metrics.get("map_iou", float("nan")),
                             r.metrics["wall_s"]))
    return results


def _lm(r: RunResult) -> float:
    if r.gt_landmarks is None or len(r.gt_landmarks) == 0 or r.landmarks is None:
        return float("nan")
    e = landmark_errors(r.landmarks, r.gt_landmarks)
    r.metrics["lm_rmse"] = e["lm_rmse"]
    r.metrics["lm_matched"] = float(e["n_matched"])
    r.metrics["n_est"] = float(e["n_est"])
    s = landmark_shape_rmse(r.landmarks, r.gt_landmarks)
    r.metrics["lm_shape"] = s["lm_shape"]
    return e["lm_rmse"]


# --------------------------------------------------------------------------- #
def summarize(results: Sequence[RunResult], outage: Optional[Tuple[float, float]] = None
              ) -> Dict[str, Dict[str, float]]:
    keys = ("ate_raw", "ate_aligned", "rpe_t", "lm_rmse", "lm_shape",
            "map_iou", "map_iou_coarse", "wall_s")
    acc: Dict[str, Dict[str, list]] = {}
    for r in results:
        if outage is not None and tuple(r.outage) != tuple(outage):
            continue
        d = acc.setdefault(r.backend, {k: [] for k in keys})
        for k in keys:
            v = r.metrics.get(k)
            if v is not None and np.isfinite(v):
                d[k].append(v)
    return {b: {k: float(np.mean(v)) for k, v in d.items() if v}
            for b, d in acc.items()}


def table(results: Sequence[RunResult], outage=(0.0, 0.0)) -> str:
    from .metrics import format_table
    s = summarize(results, outage)
    rows = []
    for b in BACKENDS:
        if b not in s:
            continue
        d = s[b]
        rows.append([BACKEND_LABEL[b],
                     "%.3f" % d.get("ate_raw", float("nan")),
                     "%.3f" % d.get("ate_aligned", float("nan")),
                     "%.3f" % d.get("rpe_t", float("nan")),
                     "%.3f" % d.get("lm_rmse", float("nan")),
                     "%.2f" % d.get("map_iou_coarse", float("nan")),
                     "%.0f" % d.get("wall_s", 0.0)])
    return format_table(rows, ["后端", "ATE 原始(m)", "ATE 对齐(m)", "RPE(m)",
                               "路标 RMSE(m)", "地图 IoU(0.5m)", "耗时(s)"])
