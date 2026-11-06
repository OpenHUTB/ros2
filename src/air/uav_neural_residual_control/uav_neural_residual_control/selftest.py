#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自检：不需要 ROS、不需要显示器，逐项验证核心模块的数值正确性。"""
from __future__ import annotations

import sys
import time
from typing import List, Tuple

import numpy as np

__all__ = ["run_selftest", "main"]


def _check(name: str, ok: bool, detail: str = "") -> Tuple[str, bool, str]:
    return name, bool(ok), detail


def run_selftest(verbose: bool = True) -> int:
    from .core import (ResidualPlant, make_plant, make_trajectory, make_tracker,
                       ResidualNet, MLP, RecursiveLeastSquares, Gains)
    from .core.disturbances import DragModel, WindModel, PayloadModel
    from .core.dataset import EpisodeCollector
    from .core.residual import build_features, FEATURE_DIM
    from .core.trajectories import TRAJECTORIES

    res: List[Tuple[str, bool, str]] = []
    t0 = time.time()

    # ---------------------------------------------------------------- 工具
    st = np.array([1.0, 2.0, -0.5])
    f = build_features(st, np.zeros(3), np.zeros(3), 1.5, 10.0)
    res.append(_check("特征维数 = %d" % FEATURE_DIM, len(f) == FEATURE_DIM, ""))
    res.append(_check("二次阻力基 vx·|v_h|",
                      abs(f[3] - 1.0 * np.hypot(1.0, 2.0)) < 1e-12, ""))
    res.append(_check("垂直二次基 vz·|vz|", abs(f[5] - (-0.5) * 0.5) < 1e-12, ""))

    # ---------------------------------------------------------------- 阻力
    d = DragModel()
    fb = d.body_force(np.array([1.0, 0.0, 0.0]))
    res.append(_check("阻力方向与速度相反", fb[0] < 0.0 and abs(fb[1]) < 1e-12,
                      "F=%.4f N" % fb[0]))
    ratio = -d.body_force(np.array([2.0, 0, 0]))[0] / -fb[0]
    res.append(_check("阻力随速度增长快于线性、慢于三次（二次项主导）",
                      3.0 < ratio < 4.0,
                      "速度翻倍时 F 变为 %.2f 倍（纯线性=2，纯二次=4）" % ratio))
    res.append(_check("阻力在 10 m/s 时约为重力的 0.5~0.9 倍",
                      0.5 < -d.body_force(np.array([10.0, 0, 0]))[0] / (0.95 * 9.81) < 0.9,
                      "%.2f 倍" % (-d.body_force(np.array([10.0, 0, 0]))[0] / (0.95 * 9.81))))

    # ---------------------------------------------------------------- 风
    w0 = WindModel(mean=(3.0, 0.0, 0.0), gust=1.8, seed=3)
    ws = np.empty((200000, 3))
    w0.reset()
    for i in range(len(ws)):                     # 2000 s @ 100 Hz
        w0.update(0.01)
        ws[i] = w0.vel
    emp = float(np.std(ws[:, 0]))
    # 相关时间 1.5 s ⇒ 有效独立样本只有 2000/1.5≈1330 个，均值/标准差的
    # 估计误差约 0.05/0.04，容差按 3σ 给
    res.append(_check("OU 过程稳态标准差 = 设定阵风 σ",
                      abs(emp - 1.8) < 0.12, "设定 1.80，实测 %.3f" % emp))
    res.append(_check("OU 过程均值 = 平均风", abs(ws[:, 0].mean() - 3.0) < 0.15,
                      "设定 3.00，实测 %.3f" % ws[:, 0].mean()))
    res.append(_check("垂直阵风按 vertical_ratio 缩小",
                      abs(np.std(ws[:, 2]) / max(emp, 1e-9) - 0.5) < 0.05,
                      "比值 %.3f" % (np.std(ws[:, 2]) / emp)))

    # ---------------------------------------------------------------- 载荷
    p_none = ResidualPlant(plant=make_plant("none", "calm"), scene="open_ground")
    p_heavy = ResidualPlant(plant=make_plant("heavy", "calm"), scene="open_ground")
    res.append(_check("无载荷时真值质量 = 标称质量",
                      abs(p_none.true_mass - p_none.nominal_mass) < 1e-9,
                      "%.4f kg" % p_none.true_mass))
    res.append(_check("+0.5 kg 载荷进入真值质量",
                      abs(p_heavy.true_mass - (p_none.true_mass + 0.5)) < 1e-6,
                      "%.4f kg" % p_heavy.true_mass))
    res.append(_check("标称参数不被载荷污染",
                      abs(p_heavy.nominal_mass - p_none.true_mass) < 1e-9,
                      "%.4f kg" % p_heavy.nominal_mass))

    # ------------------------------------------------- 残差量级与解析值一致
    plant = ResidualPlant(plant=make_plant("heavy", "calm", ground_effect=0.0),
                          scene="open_ground")
    traj = make_trajectory("hover", pos=(0.0, 0.0, 2.0), duration=6.0)
    tr = make_tracker("geo", plant, traj)
    plant.reset(pos=(0.0, 0.0, 2.0))
    for k in range(int(6.0 / plant.dt)):
        tracking_u = tr.act(k * plant.dt)
        plant.step_with_control(tracking_u)
        tr.observe()
    pred = -0.5 * 9.81 / plant.nominal_mass     # 解析垂直残差
    res.append(_check("悬停实测垂直残差 = 解析值 Δm·g/m_nom",
                      abs(tr.last_delta[2] - pred) < 0.06,
                      "解析 %.4f，实测 %.4f m/s²" % (pred, tr.last_delta[2])))
    res.append(_check("悬停静差 = |Δa|/kp_z",
                      abs(np.linalg.norm(plant.state().pos - traj.eval(5.99)[0])
                          - abs(pred) / 20.0) < 0.03,
                      "解析 %.3f m" % (abs(pred) / 20.0)))

    # ---------------------------------------------------------------- 轨迹
    for name in TRAJECTORIES:
        tj = make_trajectory(name)
        # 用中心差分校验解析速度/加速度
        h = 1e-6
        ok_v = ok_a = True
        for t in (0.37, 1.11, 2.53):
            if t + 2 * h > tj.duration:
                continue
            p_m = tj.eval(t - h)[0]
            p_0 = tj.eval(t)[0]
            p_p = tj.eval(t + h)[0]
            v_num = (p_p - p_m) / (2 * h)
            a_num = (p_p - 2 * p_0 + p_m) / (h * h)
            ok_v &= np.allclose(v_num, tj.eval(t)[1], atol=1e-4)
            ok_a &= np.allclose(a_num, tj.eval(t)[2], atol=1e-2)
        res.append(_check("轨迹 %s 速度解析解正确" % name, ok_v, ""))
        res.append(_check("轨迹 %s 加速度解析解正确" % name, ok_a, ""))

    # ---------------------------------------------------------------- 网络
    mlp = MLP((4, 6, 3), seed=0, out_gain=1.0)
    X = np.random.default_rng(0).normal(size=(7, 4))
    Y = np.random.default_rng(1).normal(size=(7, 3))
    # 数值梯度校验反向传播
    loss0, gW, gb = _mlp_loss(mlp, X, Y)
    eps = 1e-6
    mlp.W[0][0, 0] += eps
    loss1 = _mlp_loss(mlp, X, Y)[0]
    mlp.W[0][0, 0] -= eps
    num = (loss1 - loss0) / eps
    ana = gW[0][0, 0]
    res.append(_check("反向传播梯度与数值梯度一致（相对误差 <1e-5）",
                      abs(num - ana) / max(abs(num), 1e-9) < 1e-5,
                      "数值 %.8f vs 解析 %.8f" % (num, ana)))

    net = ResidualNet(sizes=(FEATURE_DIM, 16, 16, 3), seed=0)
    res.append(_check("零初始化输出层：未训练网络残差恒为 0",
                      np.allclose(net.predict(np.zeros(FEATURE_DIM)), 0.0), ""))
    Xt = np.random.default_rng(2).normal(size=(400, FEATURE_DIM))
    Yt = np.zeros((400, 3))
    Yt[:, 2] = -5.0
    Yt[:, 0] = 0.3 * Xt[:, 0]
    net.fit_scaler(Xt)
    for _ in range(120):
        net.train_epoch(Xt, Yt, batch=64, lr=5e-3, seed=0)
    err = float(np.sqrt(np.mean((net.predict_batch(Xt) - Yt) ** 2)))
    res.append(_check("可拟合常值 + 线性残差（RMSE < 0.05）", err < 0.05,
                      "RMSE=%.4f m/s²" % err))
    res.append(_check("输出限幅生效",
                      float(np.max(np.abs(net.predict(np.full((1, FEATURE_DIM), 1e3)))))
                      <= ResidualNet.CLIP + 1e-9, ""))

    # ---------------------------------------------------------------- RLS
    rls = RecursiveLeastSquares(4, n_out=2, lam=1.0, delta=1e3)
    rng = np.random.default_rng(5)
    theta_true = rng.normal(size=(4, 2))
    for _ in range(500):
        phi = rng.normal(size=4)
        rls.update(phi, theta_true.T @ phi)
    rls_err = float(np.linalg.norm(rls.theta - theta_true) / np.linalg.norm(theta_true))
    res.append(_check("RLS 收敛到真值（相对误差 < 0.02）", rls_err < 0.02,
                      "相对误差 %.2e" % rls_err))
    rls2 = RecursiveLeastSquares(3, n_out=1)
    for _ in range(20):
        rls2.update(np.zeros(3), np.ones(1))
    res.append(_check("RLS 低激励保护：能量过低的回归向量被跳过",
                      rls2.skipped == 20 and rls2.updates == 0, ""))

    # ---------------------------------------------------------------- 采集
    pl = ResidualPlant(plant=make_plant("light", "breeze", seed=1),
                       scene="open_ground")
    X, Y = EpisodeCollector(pl, excite_std=1.0, seed=1).run(
        make_trajectory("helix", turns=0.5))
    res.append(_check("采集样本数 > 100", len(X) > 100, "%d 个" % len(X)))
    res.append(_check("采集样本无 NaN/Inf",
                      bool(np.all(np.isfinite(X)) and np.all(np.isfinite(Y))), ""))
    res.append(_check("载荷残差落在解析值附近（±0.5 m/s²）",
                      abs(np.median(Y[:, 2]) - (-0.25 * 9.81 / 0.95)) < 0.5,
                      "中位数 %.3f，解析 %.3f m/s²"
                      % (np.median(Y[:, 2]), -0.25 * 9.81 / 0.95)))

    # ---------------------------------------------------------------- 控制器
    ctl_ok = True
    detail = []
    for name in ("pid", "geo"):
        pl = ResidualPlant(plant=make_plant("none", "calm"), scene="open_ground")
        tj = make_trajectory("hover", pos=(0, 0, 1.5), duration=6.0)
        tk = make_tracker(name, pl, tj)
        pl.reset(pos=(0, 0, 1.5))
        e = 0.0
        for k in range(int(6.0 / pl.dt)):
            u = tk.act(k * pl.dt)
            pl.step_with_control(u)
            tk.observe()
            e = np.linalg.norm(pl.state().pos - tj.eval(k * pl.dt)[0])
        ctl_ok &= e < 0.05
        detail.append("%s=%.4f m" % (name, e))
    res.append(_check("零扰动下 pid/geo 悬停误差 < 5 cm", ctl_ok, " ".join(detail)))

    # 推力永远落在执行器范围内
    pl = ResidualPlant(plant=make_plant("extra", "windy", seed=2), scene="open_ground")
    tj = make_trajectory("figure8")
    tk = make_tracker("pid", pl, tj)
    p0, _, _, y0, _ = tj.eval(0.0)
    pl.reset(pos=tuple(p0), yaw=float(y0))
    ok = True
    for k in range(int(tj.duration / pl.dt)):
        u = tk.act(k * pl.dt)
        ok &= bool(np.all(u >= -1e-9) and np.all(u <= pl.params.max_thrust + 1e-9))
        pl.step_with_control(u)
        tk.observe()
    res.append(_check("全推力通道始终在 [0, T_max] 内", ok, ""))

    # -------------------------------------------- 与底座实现的数值一致性
    try:
        import uav_mujoco_sim.core.quadrotor as bq
        from .core.attitude import QuadParams, mixer as my_mixer, quat_to_mat as my_q2m
        rng = np.random.default_rng(11)
        dmax = 0.0
        for _ in range(200):
            T = float(rng.uniform(0, 24))
            tau = rng.uniform(-1.5, 1.5, 3)
            bp = bq.QuadParams(mass=0.95, arm=0.106, k_m=0.015, max_thrust=6.0,
                               hover_thrust=2.33)
            mp = QuadParams(mass=0.95, arm=0.106, k_m=0.015, max_thrust=6.0,
                            hover_thrust=2.33)
            dmax = max(dmax, float(np.max(np.abs(my_mixer(T, tau, mp)
                                                 - bq.mixer(T, tau, bp)))))
            q = rng.normal(size=4)
            q /= np.linalg.norm(q)
            dmax = max(dmax, float(np.max(np.abs(my_q2m(q) - bq.quat_to_mat(q)))))
        res.append(_check("本包 attitude 与底座 core.quadrotor 数值一致", dmax < 1e-12,
                          "最大偏差 %.2e" % dmax))
    except ImportError:
        res.append(_check("与底座一致性校验（本机无 mujoco，跳过）", True, ""))

    # ---------------------------------------------------------------- 报告
    n_ok = sum(1 for _, ok_, _ in res if ok_)
    if verbose:
        print("=" * 68)
        for name, ok_, det in res:
            print("  [%s] %-52s %s" % ("OK" if ok_ else "!!", name, det))
        print("=" * 68)
        print("通过 %d/%d   用时 %.1f s" % (n_ok, len(res), time.time() - t0))
    return 0 if n_ok == len(res) else 1


def _mlp_loss(mlp, X, Y):
    Yh, hs = mlp.forward(X, cache=True)
    d = Yh - Y
    loss = float(np.mean(d ** 2))
    gW, gb = mlp.backward(X, hs, 2.0 * d / d.size)
    return loss, gW, gb


def main(argv=None) -> int:
    return run_selftest()


if __name__ == "__main__":
    sys.exit(main())
