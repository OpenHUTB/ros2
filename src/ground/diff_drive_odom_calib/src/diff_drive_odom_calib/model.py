# -*- coding: utf-8 -*-
"""里程计积分器与位姿工具（纯逻辑，不依赖 rospy、不依赖 MuJoCo）。

注意：本模块**不负责仿真**。位姿由 MuJoCo（mujoco_env.py）积分，
这里只提供"用编码器增量 + 标定系数反算位姿"的数学部分。
早期版本在这里有一个解析式 DiffDrive 类充当"仿真器"，已经删除——
那正是被指出"用六行运动学积分冒充物理仿真"的地方。

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
