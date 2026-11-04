r"""四旋翼飞行控制：位置/速度外环 + 姿态内环（串级 P / PD）。

输入是速度指令 $(v_x, v_y)$（机体系）、偏航角速度 $r$ 和高度 $z_{ref}$，
输出是作用在机体上的世界系合力 $\mathbf f$ 与机体力矩 $\boldsymbol\tau$：

$$
\mathbf a_{des}=k_v(\mathbf v_{ref}-\mathbf v),\qquad
\mathbf f_{des}=m\big(\mathbf a_{des}+g\hat z\big)
$$

$$
\mathbf b_{3}^{des}=\frac{\mathbf f_{des}}{\|\mathbf f_{des}\|},\quad
T=\min(\|\mathbf f_{des}\|,T_{max})
$$

$$
\mathbf e_R=\tfrac12\big(R_{des}^{T}R-R^{T}R_{des}\big)^{\vee},\qquad
\boldsymbol\tau=k_R\mathbf e_R-k_\omega\boldsymbol\omega
$$

这样"飞到哪里"由速度指令决定，"机身怎么倾斜"由姿态环决定，二者解耦，
上层（A* / 神经网络）只需要给速度。
"""

from __future__ import annotations

import math

import numpy as np

from .model import GRAVITY, MASS, MAX_THRUST


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


def _vee(m: np.ndarray) -> np.ndarray:
    return np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]]) * 0.5


class FlightController:
    """串级控制器；每个物理步调用一次 ``compute``。"""

    def __init__(self, mass: float = MASS, max_thrust: float = MAX_THRUST,
                 k_v: float = 2.6, k_z: float = 2.2, k_dz: float = 1.6,
                 k_r: float = 9.0, k_w: float = 1.3, max_tilt: float = 0.55):
        self.m = mass
        self.max_thrust = max_thrust
        self.k_v = k_v
        self.k_z = k_z
        self.k_dz = k_dz
        self.k_r = k_r
        self.k_w = k_w
        self.max_tilt = max_tilt
        self.yaw_ref = 0.0

    def reset(self, yaw: float = 0.0):
        self.yaw_ref = yaw

    # ------------------------------------------------------------------
    def compute(self, pos, quat, vel, omega, vx: float, vy: float, yaw_rate: float,
                z_ref: float, dt: float = 0.002):
        r = quat_to_mat(quat)
        roll, pitch, yaw = mat_to_rpy(r)
        self.yaw_ref = math.atan2(math.sin(self.yaw_ref + yaw_rate * dt),
                                  math.cos(self.yaw_ref + yaw_rate * dt))

        # 机体系速度指令 -> 世界系
        cy, sy = math.cos(yaw), math.sin(yaw)
        v_ref = np.array([cy * vx - sy * vy, sy * vx + cy * vy, 0.0])
        vz_ref = float(np.clip(self.k_z * (z_ref - pos[2]), -0.9, 0.9))
        v_ref[2] = vz_ref

        a_des = self.k_v * (v_ref - np.asarray(vel, dtype=float))
        a_des[2] = self.k_z * (z_ref - pos[2]) - self.k_dz * float(vel[2])
        f_des = self.m * (a_des + np.array([0.0, 0.0, GRAVITY]))
        if f_des[2] < 1e-3:
            f_des[2] = 1e-3

        b3 = f_des / (np.linalg.norm(f_des) + 1e-9)
        # 限制倾角，避免大速度指令把机体翻过来
        tilt = math.acos(max(-1.0, min(1.0, b3[2])))
        if tilt > self.max_tilt:
            b3 = b3 * math.sin(self.max_tilt) / max(math.sin(tilt), 1e-6)
            b3[2] = math.cos(self.max_tilt)
            b3 /= np.linalg.norm(b3)

        b1 = np.array([math.cos(self.yaw_ref), math.sin(self.yaw_ref), 0.0])
        b1 = b1 - np.dot(b1, b3) * b3
        if np.linalg.norm(b1) < 1e-6:
            b1 = np.array([1.0, 0.0, 0.0]) - b3[0] * b3
        b1 /= np.linalg.norm(b1)
        b2 = np.cross(b3, b1)
        r_des = np.stack([b1, b2, b3], axis=1)

        e_r = _vee(r_des.T @ r - r.T @ r_des)
        torque = -self.k_r * e_r - self.k_w * np.asarray(omega, dtype=float)
        torque = np.clip(torque, -1.2, 1.2)

        thrust = float(np.clip(np.linalg.norm(f_des), 0.0, self.max_thrust))
        force = r[:, 2] * thrust
        return force, torque, thrust
