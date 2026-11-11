"""逆运动学、PD(+积分) 与六肌肉协同分配。

1. 目标点 (x, z) 先做可达性检查（臂展 + 肩/肘限位）；
2. 肘上构型 IK 得到 (q1*, q2*)；
3. 肩、肘 PD + 积分给出期望关节力矩，并加重力/科氏前馈；
4. 按文档的协同关系分配到 6 条肌肉的激活：

       τ1 > 0 -> shoulder_flexor, τ1 < 0 -> shoulder_extensor
       τ2 > 0 -> elbow_flexor,    τ2 < 0 -> elbow_extensor
       共同部分 -> biarticular_flexor / biarticular_extensor

控制器不直接写关节力矩，只写激活。MuJoCo 的运动方程是

    M q̈ + qfrc_bias = qfrc_actuator

静态保持需要 τ_act = qfrc_bias，因此前馈项取 **+bias**。
"""

from __future__ import annotations

import numpy as np

from .config import Config


def inverse_kinematics(x: float, z: float, cfg: Config) -> tuple[float, float] | None:
    """肘上构型 IK；不可达、非有限或超出关节限位时返回 None。"""
    if not (np.isfinite(x) and np.isfinite(z)):
        return None
    l1, l2 = cfg.model.upper_arm, cfg.model.forearm
    r2 = x * x + z * z
    r = float(np.sqrt(r2))
    if r > l1 + l2 - 1e-9 or r < abs(l1 - l2) + 1e-9:
        return None
    cos_q2 = (r2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    cos_q2 = float(np.clip(cos_q2, -1.0, 1.0))
    q2 = float(np.arccos(cos_q2))          # 肘上：q2 > 0
    q1 = float(np.arctan2(-z, x)
               - np.arctan2(l2 * np.sin(q2), l1 + l2 * np.cos(q2)))
    if not (cfg.model.shoulder_min <= q1 <= cfg.model.shoulder_max):
        return None
    if not (cfg.model.elbow_min <= q2 <= cfg.model.elbow_max):
        return None
    return q1, q2


def allocate_activations(tau1: float, tau2: float, cfg: Config) -> np.ndarray:
    """关节力矩 -> 6 条肌肉激活（顺序见 config.MUSCLES）。"""
    mu = cfg.muscle
    t_sh = mu.f_max * mu.moment_arm_shoulder
    t_el = mu.f_max * mu.moment_arm_elbow

    sh_flex = min(max(tau1, 0.0) / t_sh, 1.0)
    sh_ext = min(max(-tau1, 0.0) / t_sh, 1.0)
    el_flex = min(max(tau2, 0.0) / t_el, 1.0)
    el_ext = min(max(-tau2, 0.0) / t_el, 1.0)

    share = cfg.ctrl.biarticular_share
    bi_flex = share * min(sh_flex, el_flex)
    bi_ext = share * min(sh_ext, el_ext)
    # 双关节肌也贡献关节力矩，单关节肌相应减少，避免总力矩过大
    sh_flex *= (1.0 - share)
    el_flex *= (1.0 - share)
    sh_ext *= (1.0 - share)
    el_ext *= (1.0 - share)

    return np.array([sh_flex, sh_ext, el_flex, el_ext, bi_flex, bi_ext],
                    dtype=float)


class ReachController:
    """带积分与重力前馈的关节空间控制器（只输出肌肉激活）。"""

    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.integral = np.zeros(2)
        self.limit = np.array([
            cfg.muscle.f_max * cfg.muscle.moment_arm_shoulder,
            cfg.muscle.f_max * cfg.muscle.moment_arm_elbow,
        ]) * 0.5

    def reset(self) -> None:
        self.integral[:] = 0.0

    def joint_target(self, target: tuple[float, float],
                     fallback: tuple[float, float]) -> tuple[float, float]:
        ik = inverse_kinematics(target[0], target[1], self.cfg)
        return fallback if ik is None else ik

    def step(self, model, target: tuple[float, float]) -> np.ndarray:
        c = self.cfg.ctrl
        q1_des, q2_des = self.joint_target(target, (model.q1, model.q2))
        e1 = q1_des - model.q1
        e2 = q2_des - model.q2

        dt = self.cfg.sim.control_period
        self.integral += np.array([e1, e2]) * dt
        self.integral = np.clip(self.integral, -self.limit, self.limit)

        bias = np.array([model.data.qfrc_bias[model.sh.dofadr[0]],
                         model.data.qfrc_bias[model.el.dofadr[0]]])
        tau = np.array([
            c.kp_shoulder * e1 + c.ki_shoulder * self.integral[0]
            - c.kd_shoulder * model.dq1,
            c.kp_elbow * e2 + c.ki_elbow * self.integral[1]
            - c.kd_elbow * model.dq2,
        ]) + bias

        acts = allocate_activations(float(tau[0]), float(tau[1]), self.cfg)
        model.set_activations(acts)
        return acts
