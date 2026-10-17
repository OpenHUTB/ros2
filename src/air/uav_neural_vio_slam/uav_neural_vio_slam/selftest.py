#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自检：不需要 ROS、不需要显示器，验证数学层与整条流水线。

    python main.py --selftest
"""
from __future__ import annotations

import math
import sys
import time
from typing import List, Tuple

import numpy as np

from .core.dataset import (VioEpisode, collect_episode, depth_to_u8, u8_to_depth,
                           DEPTH_MAX, DEPTH_MIN)
from .core.ekf_slam import EkfConfig, EkfSlam
from .core.experiment import BACKENDS, BackendConfig, run_backend
from .core.fastslam import FastSlam, FastSlamConfig
from .core.landmarks import DetectorConfig, LandmarkDetector, to_world_horizontal
from .core.mapping import GridConfig, OccupancyGrid
from .core.metrics import (align_se2, ate, landmark_errors, map_iou, poses_local,
                           rpe)
from .core.network import VioNet, gaussian_nll
from .core.nn import Adam, Conv2d, GRUCell, Linear, col2im_add, im2col
from .core.obs_model import LearnedObsModel, build_training_data
from .core.training import TrainConfig, frontend_metrics, predict_episode
from .core.world import LoopTrajectory, build_world

__all__ = ["run_selftest"]


class _Report:
    def __init__(self):
        self.n = 0
        self.n_pass = 0
        self.fail: List[str] = []

    def check(self, name: str, ok: bool, note: str = "") -> None:
        self.n += 1
        if ok:
            self.n_pass += 1
        else:
            self.fail.append(name)
        print("  [%s] %-22s %s" % ("PASS" if ok else "FAIL", name, note))


def _grad_check(layer, loss_fn, eps=1e-6, rel_tol=2e-4) -> float:
    """数值梯度 vs 解析梯度，返回最大相对误差。"""
    _, d = loss_fn()
    layer.zero_grad() if hasattr(layer, "zero_grad") else None
    layer.backward(2.0 * d / d.size)
    worst = 0.0
    for p, g in zip(layer.params, layer.grads):
        it = np.nditer(p, flags=["multi_index"])
        for _ in it:
            idx = it.multi_index
            old = p[idx]
            p[idx] = old + eps
            lp = loss_fn()[0]
            p[idx] = old - eps
            lm = loss_fn()[0]
            p[idx] = old
            num = (lp - lm) / (2 * eps)
            worst = max(worst, abs(num - g[idx]) / max(abs(num), abs(g[idx]), 1e-8))
    return worst


# --------------------------------------------------------------------------- #
def _sec_nn(rep: _Report, rng) -> None:
    print("\n[1/6] 神经网络（手写反向传播）")
    x = rng.normal(size=(2, 3, 9, 7))
    conv = Conv2d(3, 5, 3, stride=2, rng=np.random.default_rng(1))
    tgt = rng.normal(size=(2, 5, 4, 3))

    def loss_conv():
        y = conv.forward(x)
        d = y - tgt
        return float(np.mean(d * d)), d

    e = _grad_check(conv, loss_conv)
    rep.check("Conv2d 梯度", e < 2e-4, "最大相对误差 %.2e" % e)

    xg = rng.normal(size=(4, 6))
    hg = rng.normal(size=(4, 5)) * 0.3
    gru = GRUCell(6, 5, rng=np.random.default_rng(2))
    tg = rng.normal(size=(4, 5))

    def loss_gru():
        y = gru.forward(xg, hg)
        d = y - tg
        return float(np.mean(d * d)), d

    e = _grad_check(gru, loss_gru)
    rep.check("GRU 参数梯度", e < 2e-4, "最大相对误差 %.2e" % e)

    lin = Linear(4, 3, rng=np.random.default_rng(3))
    xl, tl = rng.normal(size=(7, 4)), rng.normal(size=(7, 3))

    def loss_lin():
        y = lin.forward(xl)
        d = y - tl
        return float(np.mean(d * d)), d

    e = _grad_check(lin, loss_lin)
    rep.check("Linear 梯度", e < 2e-4, "最大相对误差 %.2e" % e)

    # im2col/col2im 的伴随恒等式：前向反向互为转置的**必要**验证。
    # 只做数值梯度检查抓不到布局错位——两者同时转置时梯度检查仍然通过。
    A = rng.normal(size=(1, 2, 6, 6))
    cols = im2col(A, 3, 3, 1)
    B = rng.normal(size=cols.shape)
    lhs = float(np.sum(A * col2im_add(B, A.shape, 3, 3, 1)))
    rhs = float(np.sum(cols * B))
    rep.check("im2col 伴随恒等式", abs(lhs - rhs) < 1e-9,
              "%.8f vs %.8f" % (lhs, rhs))

    net = VioNet()
    dp, dr, ls, h = net.forward_step(rng.normal(size=(2, 2, 48, 64)),
                                     rng.normal(size=(2, 10, 6)),
                                     net.init_hidden(2))
    shape_ok = (dp.shape == (2, 3) and dr.shape == (2, 3)
                and ls.shape == (2, 6) and h.shape == (2, net.cfg.hidden))
    rep.check("VioNet 前向形状", shape_ok, "dp%s sigma%s h%s"
              % (dp.shape, ls.shape, h.shape))

    # 输出头小增益初始化：训练前增量预测应接近 0
    zero = float(np.abs(dp).max())
    rep.check("输出头小增益初始化", zero < 0.05, "初始输出最大值 %.4f" % zero)

    nll = gaussian_nll(np.zeros((4, 3)), np.zeros((4, 3)))
    rep.check("高斯 NLL", np.isfinite(nll) and abs(nll) < 1e-9,
              "单位方差零残差 NLL = %.4f" % nll)


def _sec_detector(rep: _Report, rng) -> None:
    print("\n[2/6] 路标检测器")
    det = LandmarkDetector()
    n_el, n_az = det.n_el, det.n_az
    # 合成：正前方 4 m、半径 0.10 m 的立柱
    r3 = np.full((n_el, n_az), 0.0)
    az0 = np.pi / 2                       # 机体系 x 前，方位角自 x 轴起算
    for a in range(n_az):
        ang = det.az[a]
        if abs(math.atan2(math.sin(ang - 0.0), math.cos(ang - 0.0))) < 0.02:
            for e in range(n_el):
                r3[e, a] = 4.0 / math.cos(det.elev[e])
    obs = det.detect(r3)
    rep.check("检出单根立柱", len(obs) == 1, "检出 %d 个" % len(obs))
    if obs:
        rep.check("测距误差 < 3 cm", abs(obs[0].rng - (4.0 + det.cfg.r_nominal)) < 0.03,
                  "r = %.4f m" % obs[0].rng)

    # 墙面：方位向连成一片，不应被判成立柱
    r3w = np.full((n_el, n_az), 5.0 / np.cos(det.elev)[:, None])
    rep.check("墙面不产生观测", len(det.detect(r3w)) == 0,
              "检出 %d 个" % len(det.detect(r3w)))

    # 半径补偿：观测到的是柱面，报出来的应是轴线距离
    rep.check("半径补偿", abs(det.cfg.r_nominal - 0.10) < 1e-9,
              "r_nominal = %.2f m" % det.cfg.r_nominal)

    # 三维点 → 整帧距离图的往返一致性
    pts = det.points(np.where(r3.ravel() > 0, r3.ravel(), 0.0))
    rt = det.ranges_from_points(pts)
    err = float(np.abs(rt - r3.ravel()).max())
    rep.check("点云↔距离图往返", err < 1e-9, "最大误差 %.2e" % err)

    # 姿态补偿：俯仰 15° 时只用偏航会把方位算偏
    from uav_mujoco_sim.core.quadrotor import quat_to_mat
    yaw = 0.3
    pitch = math.radians(15.0)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    q = np.array([cy * cp, sy * cp, -cy * sp, sy * sp])   # yaw 后绕 y 俯仰
    R = quat_to_mat(q)
    full = to_world_horizontal(obs, R, yaw, det.cfg.max_range)
    rep.check("全姿态转世界系", len(full) == 1 and np.isfinite(full[0].bear),
              "bear = %.4f rad" % (full[0].bear if full else float("nan")))


def _sec_slam(rep: _Report, rng) -> None:
    print("\n[3/6] SLAM 后端")
    # 完美前端 + 无噪声观测的最小场景：两个后端都应精确收敛
    world = build_world(seed=0)
    traj = LoopTrajectory(ax=6.0, ay=4.0, height=1.7, period=16.0)

    n = 160
    pos, yaw, obs_r, obs_b, obs_n = [], [], [], [], []
    for k in range(n):
        t = traj.period * k / n
        p, _, _, y, _ = traj.eval(t)
        pos.append(p)
        yaw.append(y)
        seen = []
        for lm in world.landmarks:
            dx, dy = lm.pos[0] - p[0], lm.pos[1] - p[1]
            r = math.hypot(dx, dy)
            if r > 12.0:
                continue
            # 检测器报出的是**轴线距离**（已把标称半径加回），这里按同样的
            # 约定生成，才能构成"完美观测"
            seen.append((r, math.atan2(dy, dx) - y))
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
        dp=np.zeros((n, 3), np.float32), dr=np.zeros((n, 3), np.float32),
        pos=np.asarray(pos, np.float32), yaw=np.asarray(yaw, np.float32),
        obs_r=np.asarray(obs_r, np.float32), obs_b=np.asarray(obs_b, np.float32),
        obs_n=np.asarray(obs_n, np.int32), lidar=None)
    # 从 k=1 起：k=0 时 yaw[-1] 会回绕到末帧，算出一个巨大的假增量
    for k in range(1, n):
        c, s = math.cos(ep.yaw[k - 1]), math.sin(ep.yaw[k - 1])
        d = ep.pos[k, :2] - ep.pos[k - 1, :2]
        ep.dp[k, :2] = [c * d[0] + s * d[1], -s * d[0] + c * d[1]]
        dyaw = float(ep.yaw[k] - ep.yaw[k - 1])
        ep.dr[k, 2] = math.atan2(math.sin(dyaw), math.cos(dyaw))
    ep._index = 0
    ep._gt_landmarks = world.positions
    # 前端无噪声
    pred = {"dp": ep.dp, "dr": ep.dr,
            "sigma": np.full((n, 6), 1e-4)}

    for name in ("ekf", "ekf_learned", "fastslam"):
        cfg = BackendConfig(gps_period=2.0, gps_sigma=0.05, seed=0)
        r = run_backend(ep, pred, name, cfg, make_map=False)
        rep.check("%s 收敛" % name, r.metrics["ate_raw"] < 0.05,
                  "ATE = %.4f m" % r.metrics["ate_raw"])

    # 观测噪声模型：log σ² 限幅必须在 exp 之前，否则溢出
    om = LearnedObsModel()
    om.trained = True
    for layer in om.layers:
        layer.W[:] = 1e4
        layer.b[:] = 1e4
    sr, sb = om.predict(5.0, 0.0)
    rep.check("观测模型限幅", np.isfinite(sr) and np.isfinite(sb)
              and sr <= om.cfg.sigma_r_range[1] + 1e-9,
              "σ_r = %.3f m（上限 %.2f）" % (sr, om.cfg.sigma_r_range[1]))

    # 未训练时应回退到常数
    om2 = LearnedObsModel()
    rep.check("未训练回退常数", om2.predict(3.0, 0.0) == (0.10, 0.030),
              "σ = %s" % (om2.predict(3.0, 0.0),))


def _sec_map(rep: _Report, rng) -> None:
    print("\n[4/6] 建图与指标")
    det = LandmarkDetector()
    # 一面 6 m 外的墙：射线应把它投出来
    n_az = det.n_az
    rng_img = np.zeros(det.rays())
    for a in range(n_az):
        if abs(math.atan2(math.sin(det.az[a] - np.pi / 2),
                          math.cos(det.az[a] - np.pi / 2))) < np.pi / 2:
            rng_img[a] = 6.0
    u2 = det.dirs[:, :2]
    nrm = np.linalg.norm(u2, axis=1)
    u2 = u2 / np.maximum(nrm, 1e-9)[:, None]
    g = OccupancyGrid(GridConfig())
    g.integrate(np.array([0.0, 0.0, 0.0]), rng_img * nrm * 0 + 6.0 * nrm, u2, 16.0)
    occupied = int((g.probability() > 0.5).sum())
    rep.check("射线投射产生占据格", occupied > 0, "%d 格" % occupied)
    dist = np.linalg.norm(
        np.argwhere(g.probability() > 0.5)[:, ::-1] * 0.10 - 28.0 - np.array([0, 0]),
        axis=1)
    rep.check("占据格落在 6 m 附近", float(np.median(dist)) > 4.5,
              "中位距离 %.2f m" % float(np.median(dist)))

    r = rng.normal(size=(1, 2, 6, 6))
    rep.check("im2col 形状", im2col(r, 3, 3, 1).shape[1] == 2 * 9,
              "cols = %d" % im2col(r, 3, 3, 1).shape[1])

    # SE(2) 对齐：把已知变换施加后再对齐，应还原
    P = rng.normal(size=(40, 2)) * 3.0
    th = 0.7
    Rm = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    Q = P @ Rm.T + np.array([2.0, -1.0])
    A = align_se2(np.column_stack([Q, np.zeros(40)]),
                  np.column_stack([P, np.zeros(40)]))[:, :2]
    rep.check("SE(2) 对齐还原", float(np.abs(A - P).max()) < 1e-6,
              "最大残差 %.2e" % float(np.abs(A - P).max()))

    # ATE/RPE 在已知误差下的取值
    gt = np.column_stack([np.linspace(0, 10, 50), np.zeros(50), np.zeros(50)])
    est = gt.copy()
    est[:, 0] += 0.3
    rep.check("ATE 平移 0.3 m", abs(ate(est, gt)["ate_raw"] - 0.3) < 1e-6,
              "%.4f m" % ate(est, gt)["ate_raw"])
    est2 = gt.copy()
    est2[:, 0] = gt[:, 0] * 1.001
    rep.check("RPE 对整体缩放不敏感", rpe(est2, gt, delta=10)["rpe_t"] < 0.02,
              "RPE = %.5f m" % rpe(est2, gt, delta=10)["rpe_t"])

    est_lm = np.array([[0.0, 0.0], [1.0, 0.0], [2.5, 0.0]])
    gt_lm = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [9.0, 9.0]])
    e = landmark_errors(est_lm, gt_lm)
    rep.check("路标最近邻配对", e["n_matched"] == 3 and abs(e["lm_max"] - 0.5) < 1e-9,
              "匹配 %d，最大误差 %.3f m" % (e["n_matched"], e["lm_max"]))
    rep.check("配对半径开区间",
              landmark_errors(np.array([[10.0, 0.0]]), gt_lm)["n_matched"] == 0,
              "10 m 处不应匹配")

    rep.check("地图 IoU 自比", abs(map_iou(g.probability(), g.probability()) - 1.0) < 1e-9,
              "IoU = %.4f" % map_iou(g.probability(), g.probability()))

    # 位姿局部化
    pl = poses_local(gt)
    rep.check("poses_local 固定首帧", float(np.abs(pl[0]).max()) < 1e-9,
              "首帧 = (% .3f, % .3f, % .3f)" % tuple(pl[0]))


def _sec_pipeline(rep: _Report, rng) -> None:
    print("\n[5/6] 深度图编解码与数据集")
    d = np.array([[0.0, 0.5, 3.7, 9.99, 20.0]])
    q = depth_to_u8(d)
    back = u8_to_depth(q)
    ok = (back[0, 0] == 0.0 and abs(back[0, 2] - 3.7) < 0.06
          and back[0, 4] == 0.0)
    rep.check("深度编解码往返", ok, "3.70 → %.3f m，超量程置 0" % back[0, 2])

    from .core.dataset import VioEpisode
    ep = VioEpisode(depth=np.zeros((4, 48, 64), np.uint8),
                    imu=np.zeros((4, 10, 6), np.float32),
                    dp=np.zeros((4, 3), np.float32),
                    dr=np.zeros((4, 3), np.float32),
                    pos=np.zeros((4, 3), np.float32),
                    yaw=np.zeros(4, np.float32),
                    obs_r=np.zeros((4, 12), np.float32),
                    obs_b=np.zeros((4, 12), np.float32),
                    obs_n=np.zeros(4, np.int32),
                    lidar=np.zeros((4, 540), np.uint8))
    sub = ep.subset(1, 3)
    rep.check("episode 切片", len(sub) == 2 and sub.depth.shape[0] == 2,
              "len = %d" % len(sub))


def _sec_end2end(rep: _Report, rng) -> float:
    print("\n[6/6] 端到端（短航线，不含 ROS）")
    t0 = time.time()
    world = build_world(seed=0, n_poles=12, area=8.0, n_pillars=4)
    from .core.dataset import EpisodeSpec
    spec = EpisodeSpec(0, loops=0.5, ax=6.0, ay=4.0, height=1.6, period=20.0)
    ep = collect_episode(world, spec, verbose=True)
    rep.check("采集一集", len(ep) > 50, "%d 帧" % len(ep))

    net = VioNet()
    pred = predict_episode(net, ep)
    rep.check("未训练前端可推理",
              pred["dp"].shape == (len(ep), 3) and np.all(np.isfinite(pred["dp"])),
              "dp 形状 %s" % (pred["dp"].shape,))
    m = frontend_metrics(pred, ep)
    rep.check("前端指标可算", np.isfinite(m["dp_rmse"]),
              "dp RMSE = %.4f m" % m["dp_rmse"])

    ep._index = 0
    ep._gt_landmarks = world.positions
    for backend in BACKENDS:
        cfg = BackendConfig(seed=0)
        r = run_backend(ep, pred, backend, cfg, make_map=(backend == "ekf"))
        rep.check("%s 可跑完" % backend, np.isfinite(r.metrics["ate_raw"]),
                  "ATE = %.3f m" % r.metrics["ate_raw"])
    return time.time() - t0


# --------------------------------------------------------------------------- #
def run_selftest() -> int:
    print("=" * 74)
    print("  uav_neural_vio_slam 自检")
    print("=" * 74)
    rng = np.random.default_rng(0)
    rep = _Report()
    t0 = time.time()
    _sec_nn(rep, rng)
    _sec_detector(rep, rng)
    _sec_slam(rep, rng)
    _sec_map(rep, rng)
    _sec_pipeline(rep, rng)
    dt = _sec_end2end(rep, rng)

    print("\n" + "=" * 74)
    print("  通过 %d/%d   总耗时 %.1f s" % (rep.n_pass, rep.n, time.time() - t0))
    if rep.fail:
        print("  未通过：%s" % "、".join(rep.fail))
    print("=" * 74)
    return 0 if rep.n_pass == rep.n else 1


if __name__ == "__main__":
    raise SystemExit(run_selftest())
