# -*- coding: utf-8 -*-
"""工况执行器：一侧驱动轮拖滞下沿车道直行，记录偏差 / 风险 / 补偿动作。

三档拖滞程度 × 三档车速 × 左右两侧失效，共同构成"车被拖着跑偏"的工况。
策略（mode）：
    none  不干预，复现真实跑偏过程，用于采集训练数据与对照
    rule  左右轮速差超门限才补偿（反应式）
    nn    1.0 s 前瞻跑偏概率门控补偿
"""
import math

import numpy as np

from .features import build
from .guard import FaultGuard, comp_steer
from .kinematics import differential_wheel_vels


def hold_throttle(v, v_target, cfg):
    """速度保持：目标车速做前馈，再叠加速度误差的比例修正（归一化油门）。"""
    thr = v_target / cfg.v_max + cfg.speed_keep_gain * (v_target - v)
    return float(min(1.0, max(0.0, thr)))


def run_episode(env, cfg, v_target, side="left", mode="none", net=None,
                fault_damping=None, max_s=None):
    """执行一次工况，返回逐步记录与结果指标。"""
    limit = cfg.run_max_s if max_s is None else float(max_s)
    guard = FaultGuard(cfg) if mode != "none" else None

    env.reset(fault_side=side, fault_damping=fault_damping)
    t = 0.0
    steer = 0.0
    thr_cap = 0.0
    rec = {k: [] for k in ("t", "v", "y", "yaw", "domega", "risk", "action",
                           "steer", "throttle", "gain", "feat")}
    deviate = False
    t_deviate = None
    t_first_assist = None

    while t < limit:
        v = env.forward_speed()
        thr_cap = min(1.0, thr_cap + cfg.throttle_cap_rate * cfg.dt)
        thr_cmd = min(hold_throttle(v, v_target, cfg), thr_cap)

        wl_cmd, wr_cmd = differential_wheel_vels(
            steer, thr_cmd, cfg.v_max, cfg.omega_max,
            cfg.wheel_r, cfg.track, cfg.max_wheel_omega)
        _, x = build(env, wl_cmd, wr_cmd, cfg)
        wl, wr = env.wheel_speeds()
        domega = wl - wr
        risk = float(net.predict_proba(x)) if (mode == "nn" and net is not None) else 0.0

        y = env.lateral_offset()
        yaw = env.yaw()
        yaw_deg = math.degrees(yaw)

        if mode == "none":
            gain, action = 0.0, "pass"
        else:
            gain, action = guard.decide(risk, y, yaw_deg, mode=mode)

        integ = guard.update_integ(y) if (guard is not None and gain > 0.0) else 0.0
        steer_out = comp_steer(y, yaw, env.imu()["gyro"][2], cfg, gain, integ) if gain > 0.0 else 0.0

        throttle_out = thr_cmd
        if action == "slow":
            throttle_out = thr_cmd * 0.5
        env.set_command(steer_out, throttle_out)

        rec["t"].append(t)
        rec["v"].append(v)
        rec["y"].append(y)
        rec["yaw"].append(math.degrees(yaw))
        rec["domega"].append(domega)
        rec["risk"].append(risk)
        rec["action"].append(action)
        rec["steer"].append(steer_out)
        rec["throttle"].append(throttle_out)
        rec["gain"].append(gain)
        rec["feat"].append(x)

        if not deviate and abs(y) >= cfg.deviate_m:
            deviate = True
            t_deviate = t
        if t_first_assist is None and gain > 0.0:
            t_first_assist = t
        if abs(y) >= 1.35 or env.distance() >= cfg.distance_s:
            break
        t += cfg.dt

    feat = np.asarray(rec["feat"], dtype=np.float64) if rec["feat"] else np.zeros((0, cfg.feature_dim))
    return {
        "scene": env.scene_name,
        "side": side,
        "v_target": v_target,
        "mode": mode,
        "deviate": deviate,
        "t_deviate": t_deviate,
        "t_end": t,
        "max_abs_y": max((abs(v) for v in rec["y"]), default=0.0),
        "end_y": rec["y"][-1] if rec["y"] else 0.0,
        "max_abs_yaw": max((abs(v) for v in rec["yaw"]), default=0.0),
        "assist_frames": sum(1 for a in rec["action"] if a == "assist"),
        "slow_frames": sum(1 for a in rec["action"] if a == "slow"),
        "t_first_assist": t_first_assist,
        "samples": len(rec["t"]),
        "series": rec,
        "feat": feat,
    }


def label_deviate(feat, deviate, t_deviate, cfg):
    """样本标签：越界事件窗口（越界前 horizon 到越界后 horizon）。

    正样本只覆盖"马上要越界 / 刚越界"这一小段，网络学到的是越界前兆，
    而不是"偏差已经很大"这种一眼可见的阈值。
    """
    n = feat.shape[0]
    y = np.zeros(n, dtype=np.float64)
    if not deviate or t_deviate is None:
        return y
    last = min(n - 1, int(round(t_deviate / cfg.dt)))
    start = max(0, last - cfg.horizon_steps + 1)
    end = min(n - 1, last + cfg.horizon_steps)
    y[start: end + 1] = 1.0
    return y
