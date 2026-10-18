#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四足机器人运动控制核心库（MuJoCo，纯 Python，不依赖 ROS）。

为什么需要这个文件
------------------
原模块 scripts/main_test.py 只以 100 Hz 空跑 ``mj_step`` 并打印基座位置，
``data.ctrl`` 保持默认 0；而本模型的 12 个执行器是**位置伺服**
（``biastype=affine``、``gainprm=100``），``ctrl`` 即目标关节角。ctrl=0
会被 ``ctrlrange`` 裁剪（例如 calf 被裁到上限 -0.916），实测基座高度掉到
0.059 m，机体基本趴在地上——也就是说原脚本并没有验证任何控制效果。

本库把"控制"抽象为"生成目标关节角序列"，提供两件事：
    1. 站立 / 姿态保持控制器（基线，可叠加外部推力扰动）
    2. 参数化对角步态（trot），带横滚/俯仰/基座高度三个反馈项

配套脚本：
    run_locomotion.py  跑单次实验并打印指标（可存 CSV）
    tune_gait.py       步态参数搜索 / 扫描，输出参数-稳定性对照表

用法（作为库）：
    from locomotion_core import QuadrupedModel, StandController, TrotGait, simulate

环境：Ubuntu 22.04 + MuJoCo（实测 3.16.0）+ Python 3。
"""

from __future__ import annotations

import math
import os

import mujoco
import numpy as np

# 对角步态配对：同号为一组（FR+RL 同相，FL+RR 反相）
TROT_PAIRS = {"FR": 1.0, "RL": 1.0, "FL": -1.0, "RR": -1.0}
FRONT_LEGS = {"FR", "FL"}
LEFT_LEGS = {"FR", "RL"}
LEG_ORDER = ("FR", "FL", "RR", "RL")
JOINT_PARTS = ("hip", "thigh", "calf")

FLIP_TILT_DEG = 60.0      # 超过该倾角判定为翻倒
FLIP_HEIGHT_M = 0.15      # 基座低于该高度判定为倒地


class QuadrupedModel(object):
    """加载 MJCF 模型，解析 12 个执行器与关键帧站立姿态。"""

    def __init__(self, xml_path):
        self.xml_path = xml_path
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        self.act = {}          # act[leg][part] = 执行器编号
        for leg in LEG_ORDER:
            self.act[leg] = {}
            for part in JOINT_PARTS:
                self.act[leg][part] = self._find_actuator(
                    "%s_%s_joint" % (leg, part))

        self.trunk_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "trunk")

        reset_data(self.model, self.data)
        self.home = self._home_targets()
        self.z_home = float(self.data.qpos[2])

    def _find_actuator(self, joint_name):
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if jid < 0:
            raise KeyError("模型中没有关节 %s" % joint_name)
        for i in range(self.model.nu):
            if self.model.actuator_trnid[i, 0] == jid:
                return i
        raise KeyError("关节 %s 没有对应的执行器" % joint_name)

    def _home_targets(self):
        """关键帧姿态 → 每个执行器的目标角（并按 ctrlrange 裁剪）。"""
        home = {}
        for leg in LEG_ORDER:
            home[leg] = {}
            for part in JOINT_PARTS:
                act = self.act[leg][part]
                jid = self.model.actuator_trnid[act, 0]
                lo, hi = self.model.actuator_ctrlrange[act]
                q = float(self.data.qpos[self.model.jnt_qposadr[jid]])
                home[leg][part] = float(np.clip(q, lo, hi))
        return home

    def clip(self, leg, part, value):
        """把目标关节角裁剪到执行器的 ctrlrange（与写进 ctrl 的值一致）。"""
        act = self.act[leg][part]
        lo, hi = self.model.actuator_ctrlrange[act]
        return float(np.clip(value, lo, hi))

    def write_ctrl(self, leg, part, value):
        """写入目标关节角，自动裁剪到 ctrlrange。"""
        act = self.act[leg][part]
        lo, hi = self.model.actuator_ctrlrange[act]
        self.data.ctrl[act] = float(np.clip(value, lo, hi))

    @property
    def timestep(self):
        return float(self.model.opt.timestep)


def reset_data(model, data):
    """回到关键帧站立姿态（无关键帧则回到零位）。"""
    mujoco.mj_resetData(model, data)
    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)


def base_height(data):
    return float(data.qpos[2])


def base_rpy_deg(model, data):
    """由基座四元数求横滚 / 俯仰（度）。"""
    quat = np.array(data.qpos[3:7], dtype=float)
    mat = np.zeros(9, dtype=float)
    mujoco.mju_quat2Mat(mat, quat)
    R = mat.reshape(3, 3)
    roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -R[2, 0]))))
    return roll, pitch


def tilt_deg(model, data):
    """横滚与俯仰的合成倾角（度），用于判断是否翻倒。"""
    roll, pitch = base_rpy_deg(model, data)
    return math.hypot(roll, pitch)


class StandController(object):
    """站立 / 姿态保持：把目标角固定在关键帧姿态。

    实测：从关键帧姿态出发可稳定站在 0.251 m，关节平均跟踪误差 0.018 rad。
    """

    def __init__(self, robot):
        self.robot = robot
        self.target = {leg: dict(robot.home[leg]) for leg in LEG_ORDER}
        # commanded 供 simulate() 统计关节跟踪误差
        self.commanded = self.target

    def update(self, t):
        for leg in LEG_ORDER:
            for part in JOINT_PARTS:
                self.robot.write_ctrl(leg, part, self.target[leg][part])


class TrotGait(object):
    """参数化对角步态（trot）。

    参数
    ----
    freq      步态频率 (Hz)
    a_thigh   大腿摆动幅度 (rad)
    a_calf    小腿摆动幅度 (rad)
    phase_offset  小腿相对大腿的相位差 (rad)，实测 pi/2 最好
    kp_roll   横滚反馈增益：基座横滚 → 左右腿大腿目标角
    kp_pitch  俯仰反馈增益：基座俯仰 → 前后腿大腿目标角
    kp_height 高度反馈增益：基座高度误差 → 全部大腿目标角
    a_hip     髋关节外摆幅度 (rad)，默认 0
    """

    def __init__(self, robot, freq=3.97, a_thigh=0.15, a_calf=0.29,
                 phase_offset=math.pi / 2, kp_roll=0.01, kp_pitch=0.0,
                 kp_height=0.0, a_hip=0.0):
        self.robot = robot
        self.freq = freq
        self.a_thigh = a_thigh
        self.a_calf = a_calf
        self.phase_offset = phase_offset
        self.kp_roll = kp_roll
        self.kp_pitch = kp_pitch
        self.kp_height = kp_height
        self.a_hip = a_hip
        self.commanded = {leg: dict(robot.home[leg]) for leg in LEG_ORDER}

    def params(self):
        return dict(freq=self.freq, a_thigh=self.a_thigh, a_calf=self.a_calf,
                    phase_offset=self.phase_offset, kp_roll=self.kp_roll,
                    kp_pitch=self.kp_pitch, kp_height=self.kp_height,
                    a_hip=self.a_hip)

    def update(self, t):
        robot = self.robot
        roll, pitch = base_rpy_deg(robot.model, robot.data)
        z_err = robot.z_home - base_height(robot.data)
        phase = 2.0 * math.pi * self.freq * t

        for leg, sign in TROT_PAIRS.items():
            ph = phase if sign > 0 else phase + math.pi
            d_thigh = self.a_thigh * math.sin(ph)
            d_calf = self.a_calf * math.sin(ph + self.phase_offset)

            # 姿态反馈：俯仰误差调前后腿，横滚误差调左右腿
            d_pitch = self.kp_pitch * pitch * (1.0 if leg in FRONT_LEGS else -1.0)
            d_roll = self.kp_roll * roll * (1.0 if leg in LEFT_LEGS else -1.0)
            d_height = self.kp_height * z_err

            home = robot.home[leg]
            thigh = home["thigh"] + d_thigh + d_pitch + d_roll + d_height
            calf = home["calf"] + d_calf
            hip = home["hip"] + self.a_hip * math.sin(ph)
            robot.write_ctrl(leg, "thigh", thigh)
            robot.write_ctrl(leg, "calf", calf)
            robot.write_ctrl(leg, "hip", hip)
            # 记录本次下发的目标角（与写进 ctrl 的裁剪值一致），供跟踪误差统计
            self.commanded[leg] = {
                "thigh": robot.clip(leg, "thigh", thigh),
                "calf": robot.clip(leg, "calf", calf),
                "hip": robot.clip(leg, "hip", hip),
            }


def simulate(robot, controller, seconds=4.0, disturbance=None, log=False):
    """跑一段仿真，返回指标字典（必要时返回逐帧轨迹）。

    disturbance: (牛顿, 起始秒, 持续秒) 在躯干上施加水平推力，用于抗扰测试。
    """
    model, data = robot.model, robot.data
    reset_data(model, data)
    dt = robot.timestep
    steps = int(round(seconds / dt))

    x0, y0 = float(data.qpos[0]), float(data.qpos[1])
    max_tilt = 0.0
    min_z = float("inf")
    track_err = []
    effort = 0.0
    trace = []

    for k in range(steps):
        t = k * dt
        if disturbance is not None:
            force, t_start, t_dur = disturbance
            data.xfrc_applied[:] = 0.0
            if t_start <= t < t_start + t_dur:
                data.xfrc_applied[robot.trunk_id, 0] = force
        controller.update(t)
        mujoco.mj_step(model, data)

        tilt = tilt_deg(model, data)
        z = base_height(data)
        max_tilt = max(max_tilt, tilt)
        min_z = min(min_z, z)

        # 关节跟踪误差：实际关节角 vs 本次下发的目标角
        commanded = getattr(controller, "commanded", None)
        if commanded is not None:
            for leg in LEG_ORDER:
                for part in JOINT_PARTS:
                    act = robot.act[leg][part]
                    jid = model.actuator_trnid[act, 0]
                    q = float(data.qpos[model.jnt_qposadr[jid]])
                    track_err.append(abs(q - commanded[leg][part]))
        else:
            track_err.append(0.0)

        # 能耗代理：|执行器力 × 关节速度| × dt
        for i in range(model.nu):
            jid = model.actuator_trnid[i, 0]
            if jid < 0:
                continue
            effort += abs(float(data.actuator_force[i])
                          * float(data.qvel[model.jnt_dofadr[jid]])) * dt

        if log:
            trace.append((round(t, 4), round(float(data.qpos[0]), 5),
                          round(float(data.qpos[1]), 5), round(z, 5),
                          round(tilt, 3)))

    dx = float(data.qpos[0]) - x0
    dy = float(data.qpos[1]) - y0
    flipped = (max_tilt > FLIP_TILT_DEG) or (min_z < FLIP_HEIGHT_M)
    metrics = dict(
        seconds=seconds,
        dx=dx,
        dy=dy,
        mean_speed=dx / seconds,
        max_tilt_deg=max_tilt,
        min_base_height=min_z,
        final_base_height=base_height(data),
        final_tilt_deg=tilt_deg(model, data),
        mean_track_err=float(np.mean(track_err)) if track_err else 0.0,
        energy=effort,
        flipped=bool(flipped),
    )
    if log:
        metrics["trace"] = trace
    return metrics


def default_model_path():
    """默认模型路径：本脚本同级的 ../models/quadruped.xml。"""
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, os.pardir, "models", "quadruped.xml")


def format_metrics(metrics, indent="- "):
    lines = [
        "%s前进位移 Δx: %+.3f m" % (indent, metrics["dx"]),
        "%s横向漂移 Δy: %+.3f m" % (indent, metrics["dy"]),
        "%s平均前进速度: %+.4f m/s" % (indent, metrics["mean_speed"]),
        "%s最大倾角: %.2f °" % (indent, metrics["max_tilt_deg"]),
        "%s最低基座高度: %.3f m（末态 %.3f m）"
        % (indent, metrics["min_base_height"], metrics["final_base_height"]),
        "%s关节平均跟踪误差: %.4f rad" % (indent, metrics["mean_track_err"]),
        "%s能耗代理 (Σ|τ·q̇|dt): %.2f" % (indent, metrics["energy"]),
        "%s判定: %s" % (indent, "翻倒" if metrics["flipped"] else "未翻倒"),
    ]
    return "\n".join(lines)
