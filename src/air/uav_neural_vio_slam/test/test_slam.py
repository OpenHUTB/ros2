# -*- coding: utf-8 -*-
"""SLAM 后端在构造场景下的行为。不需要 ROS、不需要 mujoco。"""
import math

import numpy as np
import pytest

from uav_neural_vio_slam.core.dataset import VioEpisode
from uav_neural_vio_slam.core.ekf_slam import EkfConfig, EkfSlam
from uav_neural_vio_slam.core.experiment import (BACKENDS, BackendConfig,
                                                 DeadReckoning, run_backend)
from uav_neural_vio_slam.core.fastslam import FastSlam, FastSlamConfig
from uav_neural_vio_slam.core.landmarks import LandmarkDetector
from uav_neural_vio_slam.core.obs_model import (LOG_CHI2_MEAN, LearnedObsModel,
                                                build_training_data)
from uav_neural_vio_slam.core.world import LoopTrajectory, build_world


def make_perfect_episode(n=160, period=16.0, ax=6.0, ay=4.0, seed=0,
                         radius=None):
    """完美前端 + 无噪声观测的闭环 episode。

    检测器报出的是**轴线距离**（它已经把标称半径加回去了），所以 ``radius=0``
    就是与观测模型完全一致的理想观测。``radius=False`` 则按真实柱面半径上报，
    留下"标称半径 vs 真实半径"的补偿残差。
    """
    world = build_world(seed=seed)
    traj = LoopTrajectory(ax=ax, ay=ay, height=1.7, period=period)
    nominal = LandmarkDetector().cfg.r_nominal
    pos, yaw, obs_r, obs_b, obs_n = [], [], [], [], []
    for k in range(n):
        p, _, _, y, _ = traj.eval(period * k / n)
        pos.append(p)
        yaw.append(y)
        seen = []
        for lm in world.landmarks:
            dx, dy = lm.pos[0] - p[0], lm.pos[1] - p[1]
            r = math.hypot(dx, dy)
            if r > 12.0:
                continue
            extra = (lm.radius - nominal) if radius is False else (
                0.0 if radius is None else radius)
            seen.append((r + extra, math.atan2(dy, dx) - y))
            if len(seen) >= 12:
                break
        rr = np.zeros(12, np.float32)
        bb = np.zeros(12, np.float32)
        for i, (a, b) in enumerate(seen):
            rr[i], bb[i] = a, b
        obs_r.append(rr)
        obs_b.append(bb)
        obs_n.append(len(seen))
    ep = VioEpisode(
        depth=np.zeros((n, 48, 64), np.uint8),
        imu=np.zeros((n, 10, 6), np.float32),
        dp=np.zeros((n, 3), np.float32),
        dr=np.zeros((n, 3), np.float32),
        pos=np.asarray(pos, np.float32), yaw=np.asarray(yaw, np.float32),
        obs_r=np.asarray(obs_r, np.float32),
        obs_b=np.asarray(obs_b, np.float32),
        obs_n=np.asarray(obs_n, np.int32), lidar=None)
    for k in range(1, n):
        c, s = math.cos(ep.yaw[k - 1]), math.sin(ep.yaw[k - 1])
        d = ep.pos[k, :2] - ep.pos[k - 1, :2]
        ep.dp[k, :2] = [c * d[0] + s * d[1], -s * d[0] + c * d[1]]
        dy = float(ep.yaw[k] - ep.yaw[k - 1])
        ep.dr[k, 2] = math.atan2(math.sin(dy), math.cos(dy))
    ep._index = 0
    ep._gt_landmarks = world.positions
    return world, ep


def perfect_pred(ep):
    return {"dp": ep.dp, "dr": ep.dr, "sigma": np.full((len(ep), 6), 1e-4)}


@pytest.mark.parametrize("backend", ["ekf", "ekf_learned", "fastslam"])
def test_backends_converge_with_perfect_frontend(backend):
    _, ep = make_perfect_episode()
    cfg = BackendConfig(gps_period=2.0, gps_sigma=0.05, seed=0)
    r = run_backend(ep, perfect_pred(ep), backend, cfg, make_map=False)
    assert r.metrics["ate_raw"] < 0.10


def test_radius_mismatch_degrades_landmarks():
    """观测用了真实半径而模型假设标称半径时，路标误差应明显变大。

    这条把"半径补偿残差"这个误差源固定下来，防止以后有人把它当成后端 bug。
    """
    from uav_neural_vio_slam.core.experiment import run_backend as rb
    from uav_neural_vio_slam.core.metrics import landmark_errors
    _, ep_ok = make_perfect_episode()
    w, ep_bad = make_perfect_episode(radius=False)
    cfg = BackendConfig(gps_period=2.0, gps_sigma=0.05, seed=0)
    a = rb(ep_ok, perfect_pred(ep_ok), "ekf", cfg, make_map=False)
    b = rb(ep_bad, perfect_pred(ep_bad), "ekf", cfg, make_map=False)
    ea = landmark_errors(a.landmarks, a.gt_landmarks)["lm_rmse"]
    eb = landmark_errors(b.landmarks, b.gt_landmarks)["lm_rmse"]
    assert eb > ea


def test_dead_reckoning_drifts():
    """同样的完美前端下，不做修正的纯积分不应有额外漂移。"""
    _, ep = make_perfect_episode()
    r = run_backend(ep, perfect_pred(ep), "odom", BackendConfig(seed=0),
                    make_map=False)
    assert r.metrics["ate_raw"] < 1e-3


def test_gps_correction_reduces_error():
    """前带偏置的前端：有 GPS 比没 GPS 好。"""
    _, ep = make_perfect_episode()
    pred = perfect_pred(ep)
    bad = {"dp": pred["dp"] + np.array([0.004, 0.0, 0.0]),
           "dr": pred["dr"], "sigma": pred["sigma"]}
    no_gps = run_backend(ep, bad, "ekf", BackendConfig(gps_start=1e9, seed=0),
                         make_map=False)
    with_gps = run_backend(ep, bad, "ekf", BackendConfig(gps_start=1.0,
                                                         gps_sigma=0.1, seed=0),
                           make_map=False)
    assert with_gps.metrics["ate_raw"] < no_gps.metrics["ate_raw"]


def test_ekf_new_landmark_covariance_symmetric_psd():
    s = EkfSlam(EkfConfig())
    lms = np.array([[3.0, 0.0], [0.0, 4.0], [-5.0, 1.0]])
    for k in range(6):
        for i, lm in enumerate(lms):
            dx, dy = lm[0] - s.pose[0], lm[1] - s.pose[1]
            r = math.hypot(dx, dy)
            b = math.atan2(dy, dx) - s.pose[2]
            s.predict(np.array([0.05, 0.0]), 0.0, None)
            s.update_landmarks(np.array([r + 0.1]), np.array([b]), 1)
    assert np.abs(s.Sigma - s.Sigma.T).max() < 1e-9
    w = np.linalg.eigvalsh(s.Sigma)
    assert w.min() > -1e-9


def test_association_gate_is_decoupled_from_obs_noise():
    """关联门限用 gate_sigma_*，与卡尔曼更新的 R 解耦。

    不解耦时，R 变准（变小）会让 S 变小、χ² 变大、门限拒绝更多真实观测，
    每个被拒的观测都会变成一个永久假路标。
    """
    _, ep = make_perfect_episode()
    det = LandmarkDetector()
    om = LearnedObsModel()
    om.trained = True
    for layer in om.layers:
        layer.W[:] = 0.0
        layer.b[:] = math.log(0.02 ** 2)          # 比常数 R 小很多
    ep._gt_landmarks = ep._gt_landmarks
    cfg = BackendConfig(seed=0)
    a = run_backend(ep, perfect_pred(ep), "ekf", cfg, make_map=False)
    b = run_backend(ep, perfect_pred(ep), "ekf_learned", cfg, obs_model=om,
                    make_map=False)
    n_real = len(ep._gt_landmarks)
    # 关联解耦后，换观测噪声不应把路标数顶到远大于真值
    assert b.metrics["n_landmarks"] <= 3 * n_real
    assert a.metrics["n_landmarks"] <= 3 * n_real


def test_fastslam_weights_normalized():
    fs = FastSlam(FastSlamConfig(n_particles=64, seed=0))
    _, ep = make_perfect_episode(n=60)
    for t in range(len(ep)):
        fs.predict(ep.dp[t], float(ep.dr[t, 2]), None)
        fs.update_landmarks(ep.obs_r[t], ep.obs_b[t], int(ep.obs_n[t]))
    assert abs(fs.w.sum() - 1.0) < 1e-9
    assert np.all(np.isfinite(fs.w))
    assert np.all(fs.w >= 0.0)


def test_obs_model_untrained_falls_back_and_clips():
    om = LearnedObsModel()
    assert om.predict(3.0, 0.0) == (0.10, 0.030)
    om.trained = True
    for layer in om.layers:
        layer.W[:] = 1e3
        layer.b[:] = 1e3
    sr, sb = om.predict(5.0, 0.0)
    assert np.isfinite(sr) and np.isfinite(sb)
    assert om.cfg.sigma_r_range[0] - 1e-9 <= sr <= om.cfg.sigma_r_range[1] + 1e-9


def test_obs_model_bias_constant_sign():
    """E[log χ²₁] = ψ(1/2)+log2 = -1.2704。写成 γ=0.5772 会让 σ 低估 √2 倍。"""
    assert abs(LOG_CHI2_MEAN - (-1.2703628)) < 1e-6
    assert LOG_CHI2_MEAN < 0
