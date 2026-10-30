# -*- coding: utf-8 -*-
"""分步标定：用两个专门设计的实验分别辨识三个误差系数。

为什么不用 UMBmark 的"双向方形路径"：
    UMBmark 用一次方形路径同时辨识两个参数，实验组织紧凑，但两个误差在同一个
    终点误差里耦合在一起，解算要用原论文的近似公式，学生不容易看懂"为什么这么算"。
    这里改用**分步标定**：直行实验只暴露轮径与跑偏，旋转实验只暴露轮距，
    两个实验各解一个方程，物理含义一目了然。代价是需要两次实验 —— 对仿真没有成本。

三个待辨识系数（定义见 model.OdometryIntegrator）：
    c_s      路程系数：c_s = Σds_true / Σds_odom
             （平均轮径偏大/偏小，走 1 m 只报 0.98 m）
    k_head   直行漂移率：k = Δθ_true / Σds_true  [rad/m]
             （左右轮不等，直行时航向持续偏）
    c_theta  转角系数：c_theta = Δθ_true / Δθ_odom
             （轮距标定错，转 90° 实际转了 92°）
"""

import math

import numpy as np

from .model import OdometryIntegrator, pose_difference, wrap_angle  # noqa: F401


def identify(straight, rotation, r_nominal=0.05, b_nominal=0.30):
    """从两组实验日志辨识 (c_s, k_head, c_theta)。

    straight / rotation 是逐帧日志列表，每帧需含：
        ds_true, dth_true            真值增量（外部测量）
        dphi_left, dphi_right        编码器增量

    三个系数的闭式解（推导见 docs/ground/diff_drive_odom_calib.md 第 4 节）：
        c_s     = Σds_true / Σds_odom
        k_head  = Δθ_true / Σds_true
        c_theta = Δθ_true / Δθ_odom
    """
    log_s = straight[0] if isinstance(straight, tuple) else straight
    ds_true = sum(o["ds_true"] for o in log_s)
    ds_odom = sum(r_nominal * 0.5 * (o["dphi_left"] + o["dphi_right"]) for o in log_s)
    dth_true = sum(o["dth_true"] for o in log_s)
    c_s = ds_true / ds_odom if abs(ds_odom) > 1e-9 else 1.0
    k_head = dth_true / ds_true if abs(ds_true) > 1e-9 else 0.0

    log_r = rotation[0] if isinstance(rotation, tuple) else rotation
    th_true = sum(o["dth_true"] for o in log_r)
    th_odom = sum(r_nominal * (o["dphi_right"] - o["dphi_left"]) / b_nominal for o in log_r)
    c_theta = th_true / th_odom if abs(th_odom) > 1e-9 else 1.0

    return dict(c_s=float(c_s), k_head=float(k_head), c_theta=float(c_theta))
