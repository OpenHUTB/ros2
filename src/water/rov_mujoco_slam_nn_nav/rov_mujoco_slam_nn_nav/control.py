r"""ROV 6-DOF 控制：速度跟踪 + 姿态保持（含浮力/阻力前馈）。

水下机器人的运动方程（体坐标系下简化为世界系各向异性阻尼）：

$$
m\dot{\mathbf v}=\mathbf T+\mathbf B-mg\hat z-\big(c\,\mathbf v+q\,\mathbf v|\mathbf v|\big)
$$

控制器直接给推力，并把浮力、重力、阻力当前值作为前馈抵消：

$$
\mathbf T=m k_v(\mathbf v_{ref}-\mathbf v)+\big(c\mathbf v+q\mathbf v|\mathbf v|\big)-\mathbf B+mg\hat z
$$

于是闭环变成一阶跟踪 $m\dot{\mathbf v}=mk_v(\mathbf v_{ref}-\mathbf v)$。
姿态用 PD 保持水平，偏航跟踪指令：

$$
\boldsymbol\tau=-k_R(\phi,\theta,0)-k_\omega\boldsymbol\omega
+k_\psi\,\mathrm{wrap}(\psi_{ref}-\psi)\hat z
$$
"""

from __future__ import annotations

import math

import numpy as np

from .model import BUOYANCY, DRAG_LIN, DRAG_QUAD, GRAVITY, MASS


def quat_to_mat(q) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_rpy(r: np.ndarray):
    roll = math.atan2(r[2, 1], r[2, 2])
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    yaw = math.atan2(r[1, 0], r[0, 0])
    return roll, pitch, yaw


def drag_force(v: np.ndarray) -> np.ndarray:
    c = np.array(DRAG_LIN)
    q = np.array(DRAG_QUAD)
    return c * v + q * v * np.abs(v)


class ROVController:
    def __init__(self, mass: float = MASS, kv: float = 3.4,
                 k_att: float = 6.0, k_w: float = 1.0, k_yaw: float = 3.0):
        self.m = mass
        self.kv = kv
        self.k_att = k_att
        self.k_w = k_w
        self.k_yaw = k_yaw
        self.yaw_ref = 0.0

    def reset(self, yaw: float = 0.0):
        self.yaw_ref = yaw

    def compute(self, pos, quat, vel, omega, cmd, dt: float):
        """cmd: dict(surge, sway, heave, yaw_rate, depth)。返回 (force_world, torque_body)。"""
        r = quat_to_mat(quat)
        roll, pitch, yaw = mat_to_rpy(r)
        self.yaw_ref = math.atan2(math.sin(self.yaw_ref + cmd["yaw_rate"] * dt),
                                  math.cos(self.yaw_ref + cmd["yaw_rate"] * dt))

        cy, sy = math.cos(yaw), math.sin(yaw)
        vx = cy * cmd["surge"] - sy * cmd["sway"]
        vy = sy * cmd["surge"] + cy * cmd["sway"]
        vz = float(np.clip(1.6 * (cmd["depth"] - pos[2]), -0.5, 0.5))
        v_ref = np.array([vx, vy, vz])

        v = np.asarray(vel, dtype=float)
        force = self.m * self.kv * (v_ref - v) + drag_force(v)
        force[2] += -BUOYANCY + self.m * GRAVITY

        yaw_err = math.atan2(math.sin(self.yaw_ref - yaw), math.cos(self.yaw_ref - yaw))
        torque = np.array([
            -self.k_att * roll - self.k_w * omega[0],
            -self.k_att * pitch - self.k_w * omega[1],
            self.k_yaw * yaw_err - self.k_w * omega[2],
        ])
        torque = np.clip(torque, -60.0, 60.0)
        force = np.clip(force, -400.0, 400.0)
        return force, torque
