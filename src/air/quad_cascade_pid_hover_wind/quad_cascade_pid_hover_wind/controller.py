#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""串级 PID 控制器：位置环 -> 姿态环 -> 角速率环 -> 旋翼混控。

状态反馈来自 /odom（位置、速度、姿态、角速度）。
输出为 4 个旋翼的目标转速。
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from .dynamics import QuadParams, thrust_to_omega
from .geometry import (body_to_world, euler_to_quat, quat_to_euler,
                       quat_to_rotation, wrap_angle, world_to_body)


@dataclass
class PIDGains:
    kp: np.ndarray
    ki: np.ndarray
    kd: np.ndarray
    i_max: np.ndarray
    out_max: np.ndarray


class PID:
    """三轴独立 PID，带积分限幅与输出限幅。"""

    def __init__(self, gains: PIDGains):
        self.g = gains
        self.integral = np.zeros(3)
        self.prev_err = np.zeros(3)
        self.has_prev = False

    def reset(self):
        self.integral = np.zeros(3)
        self.has_prev = False

    def update(self, err: np.ndarray, dt: float, deriv: Optional[np.ndarray] = None) -> np.ndarray:
        g = self.g
        self.integral += err * dt
        self.integral = np.clip(self.integral, -g.i_max, g.i_max)
        if deriv is None:
            if self.has_prev:
                d = (err - self.prev_err) / max(dt, 1e-6)
            else:
                d = np.zeros(3)
            self.prev_err = err.copy()
            self.has_prev = True
        else:
            d = deriv
        out = g.kp * err + g.ki * self.integral + g.kd * d
        return np.clip(out, -g.out_max, g.out_max)


@dataclass
class ControllerGains:
    pos: PIDGains
    att: PIDGains
    rate: PIDGains


class WaypointManager:
    """航点序列管理：到达判定 + 悬停延时。"""

    def __init__(self, waypoints: List[List[float]], acceptance: float, hover_time: float):
        self.wps = [np.array(wp, dtype=float) for wp in waypoints]
        self.acceptance = acceptance
        self.hover_time = hover_time
        self.idx = 0
        self.hover_clock = 0.0
        self.reached = False

    def current(self) -> np.ndarray:
        return self.wps[self.idx]

    def update(self, pos: np.ndarray, dt: float) -> Tuple[np.ndarray, bool]:
        """返回 (当前目标航点, 是否刚切换到下一航点)。"""
        target = self.current()
        dist = np.linalg.norm(pos - target)
        switched = False
        if dist < self.acceptance:
            if not self.reached:
                self.reached = True
                self.hover_clock = 0.0
            self.hover_clock += dt
            if self.hover_clock >= self.hover_time and self.idx < len(self.wps) - 1:
                self.idx += 1
                self.reached = False
                switched = True
        else:
            self.reached = False
        return self.current(), switched

    def finished(self, pos: np.ndarray) -> bool:
        return self.idx == len(self.wps) - 1 and \
            np.linalg.norm(pos - self.current()) < self.acceptance


class CascadeController:
    """位置-姿态-角速率 串级 PID。"""

    def __init__(self, params: QuadParams, gains: ControllerGains,
                 mixer: np.ndarray, yaw_control: bool = False, yaw_target: float = 0.0):
        self.p = params
        self.gains = gains
        self.mixer = mixer
        self.yaw_control = yaw_control
        self.yaw_target = yaw_target

        self.pos_pid = PID(gains.pos)
        self.att_pid = PID(gains.att)
        self.rate_pid = PID(gains.rate)

        # 诊断量
        self.diag = {}

    def reset(self):
        self.pos_pid.reset()
        self.att_pid.reset()
        self.rate_pid.reset()

    def _desired_attitude(self, a_des_world: np.ndarray, yaw: float) -> Tuple[float, float, float]:
        """由期望加速度（世界系）与目标偏航计算期望滚转/俯仰/偏航。"""
        g = self.p.gravity
        ax, ay, az = a_des_world
        # 期望推力方向（世界系）
        fx, fy = ax, ay
        fz = az + g
        fz = max(fz, 1e-3)
        # 小角度近似：phi = (ax*sin(yaw) - ay*cos(yaw)) / fz
        # theta = (ax*cos(yaw) + ay*sin(yaw)) / fz
        phi = (fx * math.sin(yaw) - fy * math.cos(yaw)) / fz
        theta = (fx * math.cos(yaw) + fy * math.sin(yaw)) / fz
        phi = max(-0.5, min(0.5, phi))
        theta = max(-0.5, min(0.5, theta))
        return phi, theta, yaw

    def compute(self, pos: np.ndarray, vel: np.ndarray, quat: np.ndarray,
                omega: np.ndarray, pos_des: np.ndarray, dt: float) -> np.ndarray:
        """计算 4 个旋翼目标转速。"""
        p = self.p
        roll, pitch, yaw = quat_to_euler(quat)

        # 1. 位置环（世界系） -> 期望加速度
        pos_err = pos_des - pos
        vel_feedforward = np.zeros(3)
        # 用速度作为微分反馈（PD 形式）
        a_des = self.pos_pid.update(pos_err, dt, deriv=-vel)
        a_des = np.clip(a_des, -self.gains.pos.out_max, self.gains.pos.out_max)

        # 2. 期望推力与姿态
        g = p.gravity
        fz = a_des[2] + g
        fz = max(fz, 1.0)
        thrust_des = p.mass * fz

        yaw_des = self.yaw_target if self.yaw_control else yaw
        phi_des, theta_des, psi_des = self._desired_attitude(a_des, yaw_des)

        # 3. 姿态环（欧拉角误差）-> 期望角速率
        att_err = np.array([
            wrap_angle(phi_des - roll),
            wrap_angle(theta_des - pitch),
            wrap_angle(psi_des - yaw),
        ])
        omega_des = self.att_pid.update(att_err, dt)

        # 4. 角速率环 -> 期望力矩
        omega_err = omega_des - omega
        moments = self.rate_pid.update(omega_err, dt)

        # 5. 混控：[T, Mx, My, Mz] -> 4 个旋翼推力
        u = np.array([thrust_des, moments[0], moments[1], moments[2]])
        thrusts = self.mixer @ u
        # 推力不能为负，且要保证总推力可实现
        thrusts = np.clip(thrusts, 0.0, None)
        omega_cmd = thrust_to_omega(thrusts, p.k_f, p.min_omega, p.max_omega)

        self.diag = {
            "pos_err": pos_err.tolist(),
            "a_des": a_des.tolist(),
            "thrust_des": float(thrust_des),
            "phi_des": float(phi_des), "theta_des": float(theta_des),
            "psi_des": float(psi_des),
            "omega_des": omega_des.tolist(),
            "moments": moments.tolist(),
        }
        return omega_cmd


def build_gains_from_dict(cfg: dict) -> ControllerGains:
    def g(d):
        return PIDGains(
            kp=np.array(d["kp"], float),
            ki=np.array(d["ki"], float),
            kd=np.array(d["kd"], float),
            i_max=np.array(d["i_max"], float),
            out_max=np.array(d["out_max"], float),
        )
    return ControllerGains(pos=g(cfg["position_pid"]), att=g(cfg["attitude_pid"]),
                           rate=g(cfg["rate_pid"]))
