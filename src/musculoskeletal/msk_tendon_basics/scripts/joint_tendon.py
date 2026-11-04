# -*- coding: utf-8 -*-
"""单自由度关节 + 一对肌肉-肌腱单元（拮抗肌）。

关节轴上一边是屈肌、一边是伸肌。每条肌肉都不是直接接在关节上，而是
肌纤维 -> 肌腱 -> 关节。肌肉长度随关节角变化：

    L_flex = L_rest - r · θ      关节正转时屈肌缩短
    L_ext  = L_rest + r · θ      伸肌同时被拉长

给定关节角，两侧 MTU 长度已知，用 ``muscle_tendon.solve_fiber_length``
解出肌纤维长度和力，再算关节力矩：

    τ = r · (F_flex - F_ext) - b · θ̇

本模块的动力学是**准静态**的：每一步都假设肌腱/肌纤维瞬间达到力平衡
（它们的固有时间尺度是毫秒级，远快于关节运动）。这样不用引入肌纤维
等效质量，数值上也稳定，适合讲基础概念。
"""

from __future__ import annotations

try:  # 既支持包内导入，也支持测试里直接 import
    from .muscle_tendon import TendonParams, mtu_state
except ImportError:  # pragma: no cover
    from muscle_tendon import TendonParams, mtu_state

__all__ = ["JointParams", "TendonJoint", "static_torque"]


class JointParams(object):
    """关节参数。

    inertia      等效转动惯量 (kg·m²)
    damping      关节阻尼 (N·m·s/rad)
    moment_arm   肌肉力矩臂 r (m)
    rest_mtu      θ=0 时每条 MTU 的长度 (m)
    load_stiffness 外接负载弹簧刚度 (N·m/rad)，用于释放实验
    load_rest    负载弹簧零力矩角 (rad)
    """

    def __init__(self, inertia=0.05, damping=0.20, moment_arm=0.03,
                 rest_mtu=0.135, load_stiffness=6.0, load_rest=0.05):
        if inertia <= 0:
            raise ValueError("inertia 必须为正")
        if damping < 0:
            raise ValueError("damping 不能为负")
        if moment_arm <= 0:
            raise ValueError("moment_arm 必须为正")
        if rest_mtu <= 0:
            raise ValueError("rest_mtu 必须为正")
        if load_stiffness < 0:
            raise ValueError("load_stiffness 不能为负")

        self.inertia = float(inertia)
        self.damping = float(damping)
        self.moment_arm = float(moment_arm)
        self.rest_mtu = float(rest_mtu)
        self.load_stiffness = float(load_stiffness)
        self.load_rest = float(load_rest)


def _clip(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def _mtu_lengths(params, theta):
    r = params.moment_arm
    return params.rest_mtu - r * theta, params.rest_mtu + r * theta


def static_torque(params, theta, a_flex, a_ext, muscle_params=None,
                  k_t=None):
    """等长（v=0）条件下的关节力矩 (N·m)，不含阻尼与负载。"""
    if muscle_params is None:
        muscle_params = TendonParams()
    l_flex, l_ext = _mtu_lengths(params, theta)
    flex = mtu_state(l_flex, a_flex, muscle_params, k_t)
    ext = mtu_state(l_ext, a_ext, muscle_params, k_t)
    return params.moment_arm * (flex["force"] - ext["force"])


class TendonJoint(object):
    """带一对肌肉-肌腱单元的关节，准静态肌肉、二阶关节动力学。"""

    def __init__(self, params=None, muscle_params=None,
                 theta=0.0, theta_dot=0.0):
        self.p = params if params is not None else JointParams()
        self.mp = muscle_params if muscle_params is not None else TendonParams()
        self.theta = float(theta)
        self.theta_dot = float(theta_dot)
        self.a_flex = 0.0
        self.a_ext = 0.0
        self.history = {
            "t": [], "theta": [], "theta_dot": [], "a_flex": [], "a_ext": [],
            "f_flex": [], "f_ext": [], "tau": [], "energy": [],
            "strain_flex": [], "strain_ext": [],
        }

    # -- 运动学 ----------------------------------------------------------
    def mtu_lengths(self, theta=None):
        return _mtu_lengths(self.p, self.theta if theta is None else theta)

    def muscles(self, theta=None, k_t=None):
        """返回两侧 MTU 的状态 dict，不推进时间。"""
        l_flex, l_ext = self.mtu_lengths(theta)
        flex = mtu_state(l_flex, self.a_flex, self.mp, k_t)
        ext = mtu_state(l_ext, self.a_ext, self.mp, k_t)
        return flex, ext

    def energy(self, theta=None, k_t=None):
        flex, ext = self.muscles(theta, k_t)
        return flex["energy"] + ext["energy"]

    # -- 动力学 ----------------------------------------------------------
    def step(self, u_flex, u_ext, dt, load_torque=0.0, k_t=None):
        """推进一个时间步，返回本步状态。

        参数
        ----
        u_flex, u_ext : 屈肌/伸肌神经指令 [0, 1]
        dt : 时间步 (s)
        load_torque : 外加力矩 (N·m)，正值使关节正转
        k_t : 肌腱刚度，默认用 muscle_params 的名义值
        """
        if dt <= 0:
            raise ValueError("dt 必须为正")

        p = self.p
        self.a_flex = _clip(
            _next_activation(u_flex, self.a_flex, dt, self.mp), 0.0, 1.0)
        self.a_ext = _clip(
            _next_activation(u_ext, self.a_ext, dt, self.mp), 0.0, 1.0)

        flex, ext = self.muscles(k_t=k_t)
        tau_muscle = p.moment_arm * (flex["force"] - ext["force"])
        tau = tau_muscle - p.damping * self.theta_dot + load_torque

        self.theta_dot += tau / p.inertia * dt
        self.theta += self.theta_dot * dt

        state = {
            "theta": self.theta,
            "theta_dot": self.theta_dot,
            "a_flex": self.a_flex,
            "a_ext": self.a_ext,
            "f_flex": flex["force"],
            "f_ext": ext["force"],
            "tau": tau_muscle,
            "energy": flex["energy"] + ext["energy"],
            "strain_flex": flex["tendon_strain"],
            "strain_ext": ext["tendon_strain"],
        }
        return state

    def record(self, t, state):
        h = self.history
        h["t"].append(t)
        for key in ("theta", "theta_dot", "a_flex", "a_ext",
                    "f_flex", "f_ext", "tau", "energy",
                    "strain_flex", "strain_ext"):
            h[key].append(state[key])


def _next_activation(u, a, dt, params):
    """一阶激活滞后，单独拆出来便于测试。"""
    u = _clip(float(u), 0.0, 1.0)
    a = _clip(float(a), 0.0, 1.0)
    tau = params.tau_act if u > a else params.tau_deact
    return _clip(a + dt * (u - a) / tau, 0.0, 1.0)
