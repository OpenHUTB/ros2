# -*- coding: utf-8 -*-
"""工况执行器：加速到目标车速 -> 转向斜坡 -> 记录姿态/风险/动作。

三档质心高度 × 三档车速 × 一条转向斜坡，共同构成"越转越急直到侧翻"的工况。
策略（mode）：
    none  不干预，复现真实侧翻过程，用于采集训练数据与对照
    rule  传统横滚角门限减速
    nn    0.5s 前瞻风险概率门控
"""
import math

import numpy as np

from .features import build
from .guard import RolloverGuard, rule_gate


def hold_throttle(v, v_target, cfg):
    """简单速度保持：前馈 + 比例修正。"""
    thr = 0.55 + cfg.speed_keep_gain * (v_target - v)
    return float(min(1.0, max(0.0, thr)))


def run_episode(env, cfg, v_target, mode="none", net=None, max_s=None,
                steer_ramp=None, collect=False):
    """执行一次工况，返回逐步记录与结果指标。"""
    ramp = cfg.steer_ramp_rate if steer_ramp is None else float(steer_ramp)
    limit = cfg.run_max_s if max_s is None else float(max_s)
    guard = RolloverGuard(cfg) if mode == "nn" else None

    env.reset()
    t = 0.0
    steer = 0.0
    thr_cap = 0.0
    rec = {k: [] for k in ("t", "v", "roll", "tilt", "ay", "throttle", "steer",
                           "risk", "action", "feat")}
    rollover = False
    t_roll = None

    # 阶段一：直行加速（油门缓升，避免起步瞬间后仰翘头）
    while t < cfg.accel_timeout_s:
        v = env.forward_speed()
        if v >= 0.95 * v_target:
            break
        thr_cap = min(1.0, thr_cap + cfg.throttle_ramp * cfg.dt)
        thr = min(hold_throttle(v, v_target, cfg), thr_cap)
        _, x = build(env, 0.0, cfg)
        env.set_command(0.0, thr)
        rec["t"].append(t)
        rec["v"].append(v)
        rec["roll"].append(math.degrees(env.roll_pitch(env.imu()["quat"])[0]))
        rec["tilt"].append(env.tilt_deg())
        rec["ay"].append(float(env.imu()["acc"][1]))
        rec["throttle"].append(thr)
        rec["steer"].append(0.0)
        rec["risk"].append(0.0)
        rec["action"].append("accel")
        rec["feat"].append(x)
        t += cfg.dt

    # 阶段二：转向斜坡
    while t < limit:
        v = env.forward_speed()
        thr_cap = min(1.0, thr_cap + cfg.throttle_ramp * cfg.dt)
        thr_cmd = min(hold_throttle(v, v_target, cfg), thr_cap)
        _, x = build(env, steer, cfg)
        risk = 0.0
        action = "pass"
        thr = thr_cmd
        if mode == "nn":
            risk = float(net.predict_proba(x)) if net is not None else 0.0
            steer_i, thr, action = guard.gate(risk, steer, thr_cmd)
        elif mode == "rule":
            roll_deg = math.degrees(env.roll_pitch(env.imu()["quat"])[0])
            thr, action = rule_gate(roll_deg, thr_cmd)
        env.set_command(steer, thr)

        rec["t"].append(t)
        rec["v"].append(v)
        rec["roll"].append(math.degrees(env.roll_pitch(env.imu()["quat"])[0]))
        rec["tilt"].append(env.tilt_deg())
        rec["ay"].append(float(env.imu()["acc"][1]))
        rec["throttle"].append(thr)
        rec["steer"].append(steer)
        rec["risk"].append(risk)
        rec["action"].append(action)
        rec["feat"].append(x)

        if env.is_rollover():
            rollover = True
            t_roll = t
            break
        steer = min(1.0, steer + ramp * cfg.dt)
        t += cfg.dt

    feat = np.asarray(rec["feat"], dtype=np.float64) if rec["feat"] else np.zeros((0, cfg.feature_dim))
    return {
        "scene": env.scene_name,
        "v_target": v_target,
        "mode": mode,
        "rollover": rollover,
        "t_roll": t_roll,
        "t_end": t,
        "max_roll": max(abs(r) for r in rec["roll"]) if rec["roll"] else 0.0,
        "max_tilt": max(rec["tilt"]) if rec["tilt"] else 0.0,
        "max_ay": max((abs(a) for a in rec["ay"]), default=0.0),
        "samples": len(rec["t"]),
        "series": rec,
        "feat": feat,
    }


def label_rollover(feat, rollover, t_roll, cfg):
    """按"未来 horizon 内是否翻车"给每个样本打标签。"""
    n = feat.shape[0]
    y = np.zeros(n, dtype=np.float64)
    if not rollover or t_roll is None:
        return y
    last = int(round(t_roll / cfg.dt)) - 1
    start = max(0, last - cfg.horizon_steps + 1)
    y[start: last + 1] = 1.0
    return y
