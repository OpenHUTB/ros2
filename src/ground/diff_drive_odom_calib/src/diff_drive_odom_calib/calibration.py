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

from .model import DiffDrive, OdometryIntegrator, pose_difference, wrap_angle  # noqa: F401


DEFAULTS = dict(c_s=1.0, c_theta=1.0, k_head=0.0)


def _run(robot, dt, cmd_seq):
    """跑一段指令，返回逐帧日志与真值/里程计轨迹（里程计用标称参数）。"""
    odom = OdometryIntegrator()
    robot.reset()
    odom.reset()
    log = []
    true_traj = [robot.pose]
    odom_traj = [odom.pose]
    for v, w, duration in cmd_seq:
        n = max(1, int(round(duration / dt)))
        for _ in range(n):
            out = robot.step(v, w, dt)
            odom.step(out["dphi_left"], out["dphi_right"])
            log.append(out)
            true_traj.append(robot.pose)
            odom_traj.append(odom.pose)
    return log, true_traj, odom_traj


def straight_log(robot, dt, distance=2.0, v=0.20):
    """直行实验：走 distance 米（按指令里程算时长）。"""
    return _run(robot, dt, [(v, 0.0, distance / v)])


def rotation_log(robot, dt, turns=3.0, w=1.00):
    """原地旋转实验：单向转 turns 圈。

    不能写成"正转半圈再反转半圈"来抵消朝向 —— 那样总转角会互相抵消成 0，
    辨识出来的 c_theta 就是 0/0 兜底值 1.0（实测正是如此，导致标定后
    航向仍残留 7.2°，正好是 4 次 90° 转角各差 2% 的累积）。
    """
    return _run(robot, dt, [(0.0, w, turns * 2.0 * math.pi / w)])


def identify(straight, rotation, r_nominal=0.05, b_nominal=0.30):
    """从两组实验日志辨识 (c_s, k_head, c_theta)。

    straight / rotation 既可以是 `_run()` 返回的三元组，也可以直接是逐帧日志列表，
    这样 ROS 节点就能把话题采样拼出的日志直接传进来。
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


def evaluate(robot, params, dt, cmd_seq, r_nominal=0.05, b_nominal=0.30):
    """用给定标定参数重放一段路径，返回终点误差与轨迹。

    注意：真值只用于评价（相当于现场拿卷尺量终点），
    真实系统里这一步由人工测量完成。
    """
    log, true_traj, _ = _run(robot, dt, cmd_seq)
    odom = OdometryIntegrator(c_s=params.get("c_s", 1.0),
                              c_theta=params.get("c_theta", 1.0),
                              k_head=params.get("k_head", 0.0),
                              r_nominal=r_nominal, b_nominal=b_nominal)
    odom.reset()
    est_traj = [odom.pose]
    for o in log:
        odom.step(o["dphi_left"], o["dphi_right"])
        est_traj.append(odom.pose)
    ex, ey, eth = pose_difference(odom.pose, robot.pose)
    # 路径长度（真值）：用于把位置误差归一化成百分比
    path = 0.0
    for i in range(1, len(true_traj)):
        path += math.hypot(true_traj[i][0] - true_traj[i - 1][0],
                           true_traj[i][1] - true_traj[i - 1][1])
    return dict(err_x=ex, err_y=ey, err_theta=eth,
                err_pos=math.hypot(ex, ey), path_len=path,
                true_traj=true_traj, est_traj=est_traj)


def square_cmd(side=4.0, v=0.20, w=1.00, turns=4, clockwise=True):
    """方形测试路径：每边直行 side 米，再转 90°。"""
    seg = []
    turn_time = (math.pi / 2.0) / w
    sign = -1.0 if clockwise else 1.0
    for _ in range(turns):
        seg.append((v, 0.0, side / v))
        seg.append((0.0, sign * w, turn_time))
    return seg


def repeat_test(robot, params, dt, cmd_seq, n=8, seed0=100, **kw):
    """重复实验：评估非系统误差（打滑）造成的散布，标定消不掉它。"""
    res = []
    for i in range(n):
        robot.rng = np.random.RandomState(seed0 + i)
        r = evaluate(robot, params, dt, cmd_seq, **kw)
        res.append(r)
    pos = np.array([r["err_pos"] for r in res])
    th = np.array([abs(r["err_theta"]) for r in res])
    return dict(mean_pos=float(pos.mean()), std_pos=float(pos.std()),
                mean_theta=float(th.mean()), std_theta=float(th.std()),
                n=n)
