#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""吊挂负载四旋翼仿真核心（无 ROS 依赖，可离线运行）。

物理模型见 ``models/slung_quadrotor.xml``：四旋翼 + 刚性连杆绳缆 + 球形负载。
本模块只负责“推进仿真、读写状态、量测指标”，控制律在 ``controllers.py``。
"""

import math
import os

import mujoco
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
#: 源码布局（scripts/../models）与安装布局（lib/<pkg>/../share/<pkg>/models）都要能找到
_MODEL_CANDIDATES = (
    os.path.join(_HERE, "..", "models", "slung_quadrotor.xml"),
    os.path.join(_HERE, "..", "share", "uav_slung_load", "models", "slung_quadrotor.xml"),
)
MODEL_PATH = next((p for p in _MODEL_CANDIDATES if os.path.isfile(p)), _MODEL_CANDIDATES[0])

#: 物理步长（与 MJCF 的 option timestep 一致）
DT = 0.0025
#: 控制周期：每 4 个物理步更新一次推力（100 Hz）
CTRL_DT = 0.01
#: 绳缆长度（m），ZV 整形时滞由它决定
CABLE_LENGTH = 0.5
#: 摆的自然周期 T = 2π√(L/g)
SWING_PERIOD = 2.0 * math.pi * math.sqrt(CABLE_LENGTH / 9.81)
#: 桨臂与反扭矩系数（与 MJCF 注释一致）
ARM = 0.106
K_M = 0.005
#: 单桨最大推力（N）
MAX_THRUST = 6.0
#: 机体转动惯量（与 MJCF 的 diaginertia 一致）
INERTIA = (0.0042, 0.0042, 0.0078)
#: 负载球半径（m），改负载质量时按实心球 0.4·m·r² 同步重算惯量
PAYLOAD_RADIUS = 0.06


class SlungSim:
    """吊挂负载四旋翼：``step()`` 每调用一次推进一个物理步。"""

    def __init__(self, payload_mass=0.3, model_path=MODEL_PATH):
        self.model = mujoco.MjModel.from_xml_path(os.path.abspath(model_path))
        self.data = mujoco.MjData(self.model)
        self.payload_mass = float(payload_mass)

        self.base_id = self.model.body("base_link").id
        self.payload_id = self.model.body("payload").id
        self.cable_id = self.model.body("cable").id
        self.swing_x_q = self.model.joint("swing_x").qposadr[0]
        self.swing_y_q = self.model.joint("swing_y").qposadr[0]
        self.swing_x_v = self.model.joint("swing_x").dofadr[0]
        self.swing_y_v = self.model.joint("swing_y").dofadr[0]
        self.n_u = self.model.nu

        self.reset()

    # ------------------------------------------------------------------ 状态
    @property
    def time(self):
        return float(self.data.time)

    @property
    def base_pos(self):
        return self.data.qpos[:3].copy()

    @property
    def base_vel(self):
        return self.data.qvel[:3].copy()

    @property
    def payload_pos(self):
        return self.data.xpos[self.payload_id].copy()

    @property
    def payload_vel(self):
        """负载质心速度（世界系）。"""
        v = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                 self.payload_id, v, 0)
        return v[3:6].copy()

    @property
    def cable_dir(self):
        """绳缆方向（世界系，从挂点指向负载，单位向量）。"""
        d = self.payload_pos - self.data.xpos[self.cable_id]
        n = float(np.linalg.norm(d))
        return d / n if n > 1e-9 else np.array([0.0, 0.0, -1.0])

    @property
    def swing_angle(self):
        """负载偏离铅垂线的夹角（rad），标量。"""
        return math.acos(max(-1.0, min(1.0, -float(self.cable_dir[2]))))

    @property
    def swing_rate(self):
        return np.array([self.data.qvel[self.swing_x_v], self.data.qvel[self.swing_y_v]])

    @property
    def euler(self):
        """机体滚转/俯仰/偏航（rad）。"""
        w, x, y, z = self.data.qpos[3:7]
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return roll, pitch, yaw

    @property
    def omega(self):
        return self.data.qvel[3:6].copy()

    @property
    def total_mass(self):
        return float(self.model.body_subtreemass[self.base_id])

    # ---------------------------------------------------------------- 生命周期
    def reset(self, payload_mass=None):
        """复位到悬停点；``payload_mass`` 可在运行时改负载质量（用于扫描）。"""
        if payload_mass is not None:
            self.payload_mass = float(payload_mass)
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:7] = [0.0, 0.0, 1.5, 1.0, 0.0, 0.0, 0.0]
        self.model.body_mass[self.payload_id] = self.payload_mass
        # 质量变了，负载球惯量也要同步（实心球 I = 0.4·m·r²），否则三种负载
        # 的对比就不是同一套物理（0.1 kg 的惯量会变成其实际值的 3 倍）
        inertia = 0.4 * self.payload_mass * PAYLOAD_RADIUS ** 2
        self.model.body_inertia[self.payload_id][:] = [inertia, inertia, inertia]
        # 改质量后刷新派生量（子树质量/质心），否则与编译期缓存不一致
        mujoco.mj_setConst(self.model, self.data)
        self.data.xfrc_applied[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self._next_ctrl = 0.0
        self._thrust = np.array([0.4 * self.total_mass * 9.81] * 4)

    # ------------------------------------------------------------------ 推进
    def step(self, target, controller, gust=None):
        """推进一个物理步。

        target: 三维目标位置（世界系）；controller: 实现 ``command(sim, target)``；
        gust: 可选 ``(body_id, force_xyz, t0, duration)``，用于阵风扰动实验。
        """
        if self.data.time + 1e-9 >= self._next_ctrl:
            self._thrust, tau_z = controller.command(self, target)
            self.data.ctrl[:] = self._thrust
            self.data.xfrc_applied[self.base_id][:] = [0.0, 0.0, 0.0, 0.0, 0.0, tau_z]
            self._next_ctrl += CTRL_DT

        if gust is not None:
            body_id, force, t0, duration = gust
            self.data.xfrc_applied[body_id][:3] = force if t0 <= self.time < t0 + duration else 0.0

        mujoco.mj_step(self.model, self.data)


def run_episode(controller, target, seconds, payload_mass=0.3, gust=None, sim=None,
                step_time=0.0):
    """跑一段定长仿真，返回逐步记录（位置、摆角、姿态、推力）。

    ``step_time`` 之前保持起点（悬停），到时刻后切换到 ``target``，用于阶跃响应实验。
    """
    sim = sim or SlungSim(payload_mass=payload_mass)
    sim.reset(payload_mass)
    controller.reset()
    hold = sim.base_pos.copy()
    n_steps = int(round(seconds / DT))
    log = {"t": [], "pos": [], "target": [], "swing": [],
           "payload": [], "euler": [], "thrust": []}
    for _ in range(n_steps):
        tgt = hold if sim.time < step_time else np.asarray(target, dtype=float)
        sim.step(tgt, controller, gust=gust)
        log["t"].append(sim.time)
        log["pos"].append(sim.base_pos)
        log["target"].append(tgt)
        log["swing"].append(sim.swing_angle)
        log["payload"].append(sim.payload_pos)
        log["euler"].append(sim.euler)
        log["thrust"].append(sim._thrust.copy())
    return {k: np.asarray(v) for k, v in log.items()}


def summarize(log, settle_window=2.0, t_start=1.0):
    """把一段记录折算成可对比的指标（角度均为度）。

    ``t_start`` 之前的样本不计入摆角统计（通常是阶跃前的悬停段）。
    """
    t = log["t"]
    err = np.linalg.norm(log["pos"] - log["target"], axis=1)
    mask = t >= t_start
    swing = np.degrees(log["swing"])[mask]
    err_m = err[mask]
    t_m = t[mask]
    tail = t_m >= (t_m[-1] - settle_window)
    residual = float(np.sqrt(np.mean(swing[tail] ** 2)))
    # 稳态位置误差（最后 settle_window 秒），阶跃过程中的大误差不计入
    steady_rms = float(np.sqrt(np.mean(err_m[tail] ** 2)))
    # 位置进入 ±5 cm 并保持的时刻（相对阶跃时刻）
    inside = err_m <= 0.05
    settle_idx = None
    for i in range(len(inside)):
        if inside[i:].all():
            settle_idx = i
            break
    settle = float(t_m[settle_idx] - t_start) if settle_idx is not None else float("nan")
    return {
        "swing_rms_deg": float(np.sqrt(np.mean(swing ** 2))),
        "swing_max_deg": float(np.max(swing)),
        "residual_deg": residual,
        "pos_rms_m": steady_rms,
        "pos_final_m": float(err_m[-1]),
        "settle_s": settle,
        "thrust_max_n": float(np.max(np.asarray(log["thrust"])[mask])),
    }
