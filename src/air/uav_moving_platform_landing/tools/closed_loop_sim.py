#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""移动平台降落的闭环仿真器（不含 ROS、不含仿真器，纯运动学）。

用途有两个：

* 本地回归测试（``tests/test_uav_moving_platform_landing_local.py``）用它跑完整降落；
* ``tools/evaluate.py`` 用它做「朴素追实测位置 / 追平滑当前位置 / 预测落点」
  三种策略的定量对比，结论图表都从这里来。

模型刻意做得**比真实链路保守一点**，这样结论不会靠乐观假设撑着：

* 无人机是**一阶速度滞后**（时间常数 0.35 s 水平 / 0.30 s 垂直），与
  ``carlair_ros_bridge`` 用 ``moveByVelocityBodyFrameAsync`` 时的阶跃响应同量级；
* 平台观测 10 Hz 下发，**带 ``latency`` 的延迟**（``header.stamp`` 是旧时刻），
  再叠加高斯噪声，和视觉/RPC 取数的实际情况一致；
* 控制频率 10 Hz、物理积分 100 Hz，中间不存在"控制器偷看真值"的路径 ——
  控制器只能拿到带噪带延迟的观测。
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

import moving_landing_core as core  # noqa: E402

#: 三种对照策略：只差「落点取哪一刻的平台位置」和「有没有速度前馈」
STRATEGIES = {
    # 朴素做法：直接追最近一次**实测**的平台位置，不作任何补偿。
    # 为了公平，不把它做成"增益不足"的稻草人：kp 提到 2.5，保证它能收敛到对准容差内，
    # 输就输在量测延迟和"平台会从脚下溜走"这两件事上。
    "static": {"guide/lead_mode": "measured", "guide/vff": False, "guide/kp_xy": 2.5},
    # 追平滑后的**当前**位置 + 速度前馈（把量测噪声/延迟滤掉了，但没补偿下降时间）
    "track": {"guide/lead_mode": "now", "guide/vff": True},
    # 本模块的做法：追 deck(t + T) 的相遇点 + 相遇点速度前馈
    "predict": {"guide/lead_mode": "touchdown", "guide/vff": True},
    # 模块默认配置：按平台速度/曲率/升沉自动在 now 与 touchdown 之间切换
    "auto": {"guide/lead_mode": "auto", "guide/vff": True},
}


# ---------------------------------------------------------------- 平台运动
class LineMotion(object):
    """匀速直线（车、匀速航行的船）。"""

    def __init__(self, p0=(0.0, 0.0, 0.0), v=(0.6, 0.0, 0.0)):
        self.p0 = np.asarray(p0, dtype=float)
        self.v = np.asarray(v, dtype=float)

    def pos(self, t):
        return self.p0 + self.v * t

    def vel(self, t):
        return self.v

    def name(self):
        return "line_v%.1f" % math.hypot(self.v[0], self.v[1])


class CircleMotion(object):
    """匀速圆周（转弯的车、绕圈的无人船）。"""

    def __init__(self, center=(0.0, 0.0, 0.0), radius=4.0, speed=0.6, phase=0.0):
        self.c = np.asarray(center, dtype=float)
        self.r = float(radius)
        self.w = float(speed) / float(radius)
        self.phase = float(phase)

    def pos(self, t):
        a = self.w * t + self.phase
        return self.c + np.array([self.r * math.cos(a), self.r * math.sin(a), 0.0])

    def vel(self, t):
        a = self.w * t + self.phase
        return np.array([-self.r * self.w * math.sin(a),
                         self.r * self.w * math.cos(a), 0.0])

    def name(self):
        return "circle_v%.1f" % (self.r * self.w)


class HeaveMotion(object):
    """给任意运动叠加垂向升沉（海况下的小艇、越野起伏的地面）。

    升沉是最能打穿"追当前位置"的一类扰动：平台在垂直方向来回走，
    只按 ``T = z / v`` 估剩余时间的方案会算错 0.5 s 以上。
    """

    def __init__(self, inner, amp=0.15, period=4.0, phase=0.0):
        self.inner = inner
        self.amp = float(amp)
        self.period = float(period)
        self.phase = float(phase)
        self.w = 2.0 * math.pi / self.period

    def pos(self, t):
        p = np.array(self.inner.pos(t), dtype=float)
        p[2] += self.amp * math.sin(self.w * t + self.phase)
        return p

    def vel(self, t):
        v = np.array(self.inner.vel(t), dtype=float)
        v[2] += self.amp * self.w * math.cos(self.w * t + self.phase)
        return v

    def name(self):
        return "%s_heave%.2f" % (self.inner.name(), self.amp)


# ---------------------------------------------------------------- 被控对象
class DroneModel(object):
    """一阶速度滞后的运动学模型（与 AirSim 速度通道的阶跃响应同量级）。"""

    def __init__(self, pos, tau_xy=0.35, tau_z=0.30):
        self.p = np.asarray(pos, dtype=float)
        self.v = np.zeros(3)
        self.tau_xy = float(tau_xy)
        self.tau_z = float(tau_z)

    def step(self, cmd, dt):
        cmd = np.asarray(cmd, dtype=float)
        tau = np.array([self.tau_xy, self.tau_xy, self.tau_z])
        alpha = np.clip(dt / tau, 0.0, 1.0)
        self.v = self.v + (cmd - self.v) * alpha
        self.p = self.p + self.v * dt
        return self.p


# ---------------------------------------------------------------- 观测
def measure(stamp, motion, noise_m, rng):
    """生成一条"带噪声"的平台观测；``stamp`` 是观测时刻（不是收到时刻）。

    延迟在调用侧体现为「到达时刻 = stamp + latency」。
    """
    pos = motion.pos(stamp)
    if noise_m > 0:
        pos = pos + rng.normal(0.0, noise_m, size=3)
    return core.DeckMeasurement(stamp, pos)


# ---------------------------------------------------------------- 单次试验
def run_trial(motion, strategy="predict", latency=0.15, noise_m=0.04, seed=0,
              duration=75.0, control_hz=10.0, sim_hz=100.0,
              start_offset=(0.0, 0.0, 8.0), params=None, keep_trace=True):
    """跑一次完整降落，返回指标字典（含轨迹，便于画图）。

    返回字典里 ``error_xy`` 是**接地瞬间**无人机与平台真实中心的水平距离（用仿真真值算，
    不是控制器自己的估计），``ok`` 表示落点误差是否在 0.30 m 内。
    """
    if strategy not in STRATEGIES:
        raise ValueError("未知策略 %r，可选 %s" % (strategy, sorted(STRATEGIES)))
    p = dict(core.DEFAULT_PARAMS)
    p.update(STRATEGIES[strategy])
    if params:
        p.update(params)

    rng = np.random.RandomState(int(seed))
    ctl = core.MovingLandingController(p)
    deck0 = motion.pos(0.0)
    drone = DroneModel(deck0 + np.asarray(start_offset, dtype=float))

    dt = 1.0 / float(sim_hz)
    ctrl_every = max(1, int(round(float(sim_hz) / float(control_hz))))
    n_steps = int(round(float(duration) * float(sim_hz)))

    pending = []          # (到达时刻, DeckMeasurement)
    arrived = None        # 最近一条已到达但还没被控制器消费的观测
    cmd = np.zeros(3)
    trace = {"t": [], "drone": [], "deck": [], "cmd": [], "state": [],
             "z_rel": [], "lead": [], "T": []}
    result = None
    next_measure = 0.0
    measure_period = 1.0 / float(control_hz)

    for k in range(n_steps):
        now = k * dt
        # 平台观测：按 10 Hz 生成，延迟 latency 后才到达控制器
        while now + 1e-9 >= next_measure:
            m = measure(next_measure, motion, noise_m, rng)
            pending.append((next_measure + latency, m))
            next_measure += measure_period
        # 观测到达即留存，等下一次控制节拍再交给控制器（否则 10 Hz 的样本
        # 会因为到达时刻不在控制节拍上被整个丢掉）
        while pending and pending[0][0] <= now + 1e-9:
            arrived = pending.pop(0)[1]

        if k % ctrl_every == 0:
            out = ctl.step(now, drone.p, drone.v, arrived)
            arrived = None
            cmd = np.asarray(out["cmd_vel"], dtype=float)
            if keep_trace:
                trace["t"].append(now)
                trace["drone"].append(drone.p.copy())
                trace["deck"].append(motion.pos(now))
                trace["cmd"].append(cmd.copy())
                trace["state"].append(out["state"])
                trace["z_rel"].append(drone.p[2] - motion.pos(now)[2])
                trace["lead"].append(None if out["lead"] is None else np.array(out["lead"]))
                trace["T"].append(out["time_to_touchdown"])
            if out["state"] in core.TERMINAL_STATES:
                truth = motion.pos(now)
                err = float(math.hypot(drone.p[0] - truth[0], drone.p[1] - truth[1]))
                result = {
                    "strategy": strategy,
                    "state": out["state"],
                    "landed": out["state"] == core.LANDED,
                    "aborted": out["state"] == core.ABORT,
                    "go_around": ctl.go_around,
                    "error_xy": err if out["state"] == core.LANDED else float("nan"),
                    "error_xy_last": err,
                    "duration_s": now,
                    "z_rel": float(drone.p[2] - truth[2]),
                    "reason": (None if not ctl.events else
                               next((e.get("reason") for e in reversed(ctl.events)
                                     if e.get("reason")), None)),
                    "trace": trace if keep_trace else None,
                    "motion": motion,
                    "controller": ctl,
                }
                break

        drone.step(cmd, dt)

    if result is None:          # 超时未终结
        truth = motion.pos(duration)
        err = float(math.hypot(drone.p[0] - truth[0], drone.p[1] - truth[1]))
        result = {
            "strategy": strategy, "state": "TIMEOUT", "landed": False,
            "aborted": False, "go_around": ctl.go_around,
            "error_xy": float("nan"), "error_xy_last": err,
            "duration_s": duration, "z_rel": float(drone.p[2] - truth[2]),
            "reason": "仿真时长内未结束", "trace": trace if keep_trace else None,
            "motion": motion, "controller": ctl,
        }
    result["ok"] = bool(result["landed"] and result["error_xy"] <= 0.30)
    return result


def summarize(results):
    """把一组试验压成 mean / p95 / 成功率。"""
    errs = [r["error_xy"] for r in results if r["landed"]]
    ok = sum(1 for r in results if r["ok"])
    return {
        "n": len(results),
        "landed": sum(1 for r in results if r["landed"]),
        "ok": ok,
        "rate": ok / float(len(results)) if results else 0.0,
        "mean": float(np.mean(errs)) if errs else float("nan"),
        "p95": float(np.percentile(errs, 95)) if errs else float("nan"),
        "go_around": int(sum(r["go_around"] for r in results)),
        "mean_time": float(np.mean([r["duration_s"] for r in results])),
    }
