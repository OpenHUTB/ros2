# -*- coding: utf-8 -*-
"""差速机器人的真实运动学与里程计积分（纯逻辑，不依赖 rospy）。

两条链路刻意分开：

    指令 (v, ω) ──标称 r,b 反解──> 轮速 φ̇ ──真实 r,b 积分──> 真实位姿
                        └──────────标称 r,b 积分──────────> 里程计位姿

里程计的漂移就来自"同一串 φ̇，用错了 r 和 b"。真实轮径不等 → 直行跑偏；
轮距标定错 → 转角偏。这两个误差正是标定要辨识的对象。

单位：米、弧度、秒。
"""

import math

import numpy as np


# 标称参数（机器人以为自己知道的）：轮半径 5 cm、轮距 30 cm
R_NOMINAL = 0.05
B_NOMINAL = 0.30


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _se2_step(pose, ds, dth):
    x, y, th = pose
    x += ds * math.cos(th + 0.5 * dth)
    y += ds * math.sin(th + 0.5 * dth)
    th = wrap_angle(th + dth)
    return (x, y, th)


class DiffDrive(object):
    """真实机器人：把指令轮速用**真实**参数积分成真实位姿，同时输出编码器增量。

    打滑建模成"轮子转了但地面没走那么多"，用一个 0 均值高斯扰动乘在轮子位移上，
    这是非系统误差的来源 —— 标定消不掉它，只能靠统计它的散布。
    """

    def __init__(self, r_left=0.0505, r_right=0.0495, b_true=0.306,
                 r_nominal=R_NOMINAL, b_nominal=B_NOMINAL,
                 slip_std=0.004, tick_noise=0.0, seed=0, pose=(0.0, 0.0, 0.0)):
        self.r_left = float(r_left)
        self.r_right = float(r_right)
        self.b_true = float(b_true)
        self.r_nominal = float(r_nominal)
        self.b_nominal = float(b_nominal)
        self.slip_std = float(slip_std)
        self.tick_noise = float(tick_noise)
        self.rng = np.random.RandomState(seed)
        self.pose = tuple(float(v) for v in pose)
        # 累计轮子转角（编码器读数）
        self.phi_left = 0.0
        self.phi_right = 0.0

    # ------------------------------------------------------------------
    def reset(self, pose=(0.0, 0.0, 0.0)):
        self.pose = tuple(float(v) for v in pose)
        self.phi_left = 0.0
        self.phi_right = 0.0

    def wheel_speeds(self, v, w):
        """指令 (v, ω) → 轮速，使用**标称**参数（真实驱动器的做法）。"""
        r = self.r_nominal
        b = self.b_nominal
        return (v - w * b * 0.5) / r, (v + w * b * 0.5) / r

    def step(self, v, w, dt):
        """推进一步，返回本步的真实位移、转角与编码器增量。"""
        wl, wr = self.wheel_speeds(v, w)
        # 编码器读数：轮子实际转过的角度（含少量量化噪声）
        dphi_l = wl * dt
        dphi_r = wr * dt
        if self.tick_noise > 0.0:
            dphi_l += self.rng.normal(0.0, self.tick_noise)
            dphi_r += self.rng.normal(0.0, self.tick_noise)
        self.phi_left += dphi_l
        self.phi_right += dphi_r

        # 真实位移：用真实轮径 + 打滑扰动
        slip_l = 1.0 + (self.rng.normal(0.0, self.slip_std) if self.slip_std > 0 else 0.0)
        slip_r = 1.0 + (self.rng.normal(0.0, self.slip_std) if self.slip_std > 0 else 0.0)
        ds_l = self.r_left * dphi_l * slip_l
        ds_r = self.r_right * dphi_r * slip_r
        ds = 0.5 * (ds_l + ds_r)
        dth = (ds_r - ds_l) / self.b_true
        self.pose = _se2_step(self.pose, ds, dth)
        return dict(ds_true=ds, dth_true=dth, dphi_left=dphi_l, dphi_right=dphi_r)


class OdometryIntegrator(object):
    """里程计积分器：同一串编码器增量，用（可标定的）参数积分。

        ds = c_s · r · (φ̇l + φ̇r)/2 · dt
        dθ = c_θ · r · (φ̇r − φ̇l)/b_nom · dt  +  k · ds

    三个待标定系数：
        c_s      路程标定系数（修正平均轮径）
        c_theta  转角标定系数（修正轮距）
        k        直行航向漂移率 (rad/m)，修正左右轮不等造成的跑偏
    """

    def __init__(self, c_s=1.0, c_theta=1.0, k_head=0.0,
                 r_nominal=R_NOMINAL, b_nominal=B_NOMINAL, pose=(0.0, 0.0, 0.0)):
        self.c_s = float(c_s)
        self.c_theta = float(c_theta)
        self.k_head = float(k_head)
        self.r = float(r_nominal)
        self.b = float(b_nominal)
        self.pose = tuple(float(v) for v in pose)

    def reset(self, pose=(0.0, 0.0, 0.0)):
        self.pose = tuple(float(v) for v in pose)

    def step(self, dphi_left, dphi_right):
        ds = self.c_s * self.r * 0.5 * (dphi_left + dphi_right)
        dth = self.c_theta * self.r * (dphi_right - dphi_left) / self.b + self.k_head * ds
        self.pose = _se2_step(self.pose, ds, dth)
        return ds, dth

    @property
    def params(self):
        return dict(c_s=self.c_s, c_theta=self.c_theta, k_head=self.k_head)


def pose_difference(est, true):
    """估计位姿相对真值位姿的误差 (Δx, Δy, Δθ)，位置误差换算到真值航向系下。"""
    dx = est[0] - true[0]
    dy = est[1] - true[1]
    c, s = math.cos(true[2]), math.sin(true[2])
    return (c * dx + s * dy, -s * dx + c * dy, wrap_angle(est[2] - true[2]))


def simulate(robot, integrator, cmd_seq, dt, log=None):
    """跑一段指令序列，返回 (真值轨迹, 里程计轨迹)。

    cmd_seq: [(v, w, duration_s), ...]
    """
    robot.reset()
    integrator.reset()
    true_traj = [robot.pose]
    odom_traj = [integrator.pose]
    for v, w, duration in cmd_seq:
        n = max(1, int(round(duration / dt)))
        for _ in range(n):
            out = robot.step(v, w, dt)
            integrator.step(out["dphi_left"], out["dphi_right"])
            true_traj.append(robot.pose)
            odom_traj.append(integrator.pose)
            if log is not None:
                log.append(dict(ds_true=out["ds_true"], dth_true=out["dth_true"],
                                dphi_left=out["dphi_left"], dphi_right=out["dphi_right"]))
    return true_traj, odom_traj
