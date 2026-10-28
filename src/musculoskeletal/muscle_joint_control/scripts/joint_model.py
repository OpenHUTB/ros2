# -*- coding: utf-8 -*-
"""单自由度关节和一对拮抗肌。

关节轴上一边挂一块肌肉，屈肌和伸肌。肌肉不直接出力矩，给的是激活度，力由
muscle_model 里的 Hill 模型算。

    J * theta_ddot = r * (F_flex - F_ext) - b * theta_dot

r 是肌肉在关节上的力矩臂，b 是关节阻尼。长度跟关节角的关系用小角度近似：

    l_flex = l_rest - r * theta     关节正转时屈肌缩短
    l_ext  = l_rest + r * theta     伸肌同时被拉长

只用标准库和 muscle_model，不需要 MuJoCo 或 ROS，Python 3.8 以上都能跑。
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from muscle_model import Muscle, MuscleParams  # noqa: E402

__all__ = ["JointParams", "MuscleJoint"]


class JointParams(object):
    """关节与肌肉对参数。

    属性
    ----
    inertia : 关节等效转动惯量 (kg·m²)
    damping : 关节阻尼 (N·m·s/rad)
    moment_arm : 肌肉力矩臂 r (m)
    rest_length : 两肌在关节角为 0 时的长度 (m)
    l_opt : 肌纤维最优长度 (m)，应接近 rest_length
    f_max : 单块肌肉最大等长收缩力 (N)
    v_max : 最大缩短速度 (m/s)
    """

    def __init__(self, inertia=0.02, damping=0.05, moment_arm=0.03,
                 rest_length=0.10, l_opt=0.10, f_max=120.0, v_max=1.2,
                 tau_act=0.01, tau_deact=0.04):
        if inertia <= 0:
            raise ValueError("inertia 必须为正")
        if damping < 0:
            raise ValueError("damping 不能为负")
        if moment_arm <= 0:
            raise ValueError("moment_arm 必须为正")
        if rest_length <= 0 or l_opt <= 0:
            raise ValueError("长度必须为正")

        self.inertia = float(inertia)
        self.damping = float(damping)
        self.moment_arm = float(moment_arm)
        self.rest_length = float(rest_length)
        self.l_opt = float(l_opt)
        self.f_max = float(f_max)
        self.v_max = float(v_max)
        self.tau_act = float(tau_act)
        self.tau_deact = float(tau_deact)

    def muscle_params(self):
        return MuscleParams(f_max=self.f_max, l_opt=self.l_opt,
                            v_max=self.v_max,
                            tau_act=self.tau_act, tau_deact=self.tau_deact)


class MuscleJoint(object):
    """带一对拮抗肌的单自由度关节。"""

    def __init__(self, params=None, theta=0.0, theta_dot=0.0):
        self.p = params if params is not None else JointParams()
        self.theta = float(theta)
        self.theta_dot = float(theta_dot)

        mp = self.p.muscle_params()
        self.flexor = Muscle(mp)
        self.extensor = Muscle(mp)

        #: 供绘图/分析使用的历史
        self.history = {"t": [], "theta": [], "theta_dot": [],
                        "a_flex": [], "a_ext": [], "f_flex": [], "f_ext": [],
                        "tau": []}

    # -- 运动学 ----------------------------------------------------------
    def muscle_state(self):
        """返回 (屈肌的 l_n, v_n, 伸肌的 l_n, v_n)，均已归一化。

        v_n 为**缩短速度**归一化值。注意符号：

            l_flex = l_rest - r·θ   →  dl_flex/dt = -r·θ̇
            缩短速度 = -dl/dt        →  v_flex = +r·θ̇

        因此关节正转（θ̇ > 0）时屈肌缩短（v_flex > 0），伸肌被拉长
        （v_ext = -r·θ̇ < 0，离心收缩）。
        """
        p = self.p
        r = p.moment_arm
        l_flex = p.rest_length - r * self.theta
        l_ext = p.rest_length + r * self.theta
        v_flex = r * self.theta_dot         # 正转 -> 屈肌缩短(正)
        v_ext = -r * self.theta_dot         # 正转 -> 伸肌拉长(负)

        return (l_flex / p.l_opt, v_flex / p.v_max,
                l_ext / p.l_opt, v_ext / p.v_max)

    # -- 动力学 ----------------------------------------------------------
    def step(self, u_flex, u_ext, dt, tau_ext=0.0):
        """按神经指令推进一个时间步，返回本步的关节力矩。

        参数
        ----
        u_flex, u_ext : 屈肌/伸肌的神经指令 [0, 1]
        dt : 时间步 (s)
        tau_ext : 外加力矩 (N·m)，正值使关节正转，用于扰动/负载测试

        返回
        ----
        float : 肌肉产生的关节力矩（不含 tau_ext）
        """
        if dt <= 0:
            raise ValueError("dt 必须为正")

        p = self.p
        lf, vf, le, ve = self.muscle_state()

        # 先按当前长度/速度算力（用推进前的激活度）
        f_flex = self.flexor.force(lf, vf)
        f_ext = self.extensor.force(le, ve)

        tau = p.moment_arm * (f_flex - f_ext) - p.damping * self.theta_dot
        theta_ddot = (tau + tau_ext) / p.inertia

        # 半隐式欧拉：先更新速度再更新位置，数值上更稳定
        self.theta_dot += theta_ddot * dt
        self.theta += self.theta_dot * dt

        # 推进激活度
        self.flexor.update(u_flex, dt)
        self.extensor.update(u_ext, dt)

        return tau

    def record(self, t):
        h = self.history
        lf, vf, le, ve = self.muscle_state()
        h["t"].append(t)
        h["theta"].append(self.theta)
        h["theta_dot"].append(self.theta_dot)
        h["a_flex"].append(self.flexor.activation)
        h["a_ext"].append(self.extensor.activation)
        h["f_flex"].append(self.flexor.force(lf, vf))
        h["f_ext"].append(self.extensor.force(le, ve))
        h["tau"].append(self.p.moment_arm *
                        (h["f_flex"][-1] - h["f_ext"][-1]))

    def reset(self, theta=0.0, theta_dot=0.0):
        self.theta = float(theta)
        self.theta_dot = float(theta_dot)
        self.flexor.reset()
        self.extensor.reset()
        for key in self.history:
            self.history[key] = []


# ---------------------------------------------------------------------------
def simulate_step_response(target, duration=1.5, dt=0.001, kp=9.0, kd=0.9,
                           co_activation=0.02, joint=None, joint_params=None):
    """阶跃指令下的关节角响应。

    控制器把 PD 算出的期望力矩**分配**给两块肌肉：

    * 需要正力矩 -> 屈肌激活，伸肌维持一个很小的共激活
    * 需要负力矩 -> 伸肌激活，屈肌维持共激活

    这正是肌肉驱动与力矩驱动的关键差异：控制量被限制在 [0, 1]，
    负力矩只能靠对侧肌肉产生，无法像力矩源那样"要多少给多少"。

    返回
    ----
    dict : 含 history 与若干性能指标
    """
    if dt <= 0:
        raise ValueError("dt 必须为正")
    if duration <= 0:
        raise ValueError("duration 必须为正")

    j = joint if joint is not None else MuscleJoint(joint_params)
    j.reset()

    # 把期望力矩映射为激活度：tau_max 取单块肌肉全力作用于力矩臂
    tau_max = j.p.f_max * j.p.moment_arm
    steps = int(round(duration / dt))

    for i in range(steps):
        t = i * dt
        err = target - j.theta
        tau_cmd = kp * err - kd * j.theta_dot

        # 归一化并分配到两块肌肉
        ratio = tau_cmd / tau_max
        if ratio >= 0.0:
            u_flex = co_activation + (1.0 - co_activation) * min(1.0, ratio)
            u_ext = co_activation
        else:
            u_flex = co_activation
            u_ext = co_activation + (1.0 - co_activation) * min(1.0, -ratio)

        j.step(u_flex, u_ext, dt)
        j.record(t)

    return {"history": j.history, "metrics": evaluate(j, target)}


def evaluate(joint, target):
    """从历史里算几个常用指标。"""
    h = joint.history
    if not h["t"]:
        raise ValueError("没有历史数据，请先 simulate")

    theta = h["theta"]
    t = h["t"]
    final = theta[-1]

    # 稳态误差：取最后 10% 的平均偏差
    n = max(1, len(theta) // 10)
    tail = theta[-n:]
    ss_err = sum(tail) / len(tail) - target

    # 上升时间：首次达到目标 90% 的时刻
    rise_t = None
    if abs(target) > 1e-9:
        thr = 0.9 * target
        for ti, th in zip(t, theta):
            if (target > 0 and th >= thr) or (target < 0 and th <= thr):
                rise_t = ti
                break

    # 超调
    if target > 0:
        peak = max(theta)
    else:
        peak = min(theta)
    overshoot = (peak - target) / target * 100.0 if abs(target) > 1e-9 else 0.0
    overshoot = max(0.0, overshoot)

    return {
        "final_angle": final,
        "steady_state_error": ss_err,
        "rise_time": rise_t,
        "peak_angle": peak,
        "overshoot_percent": overshoot,
        "max_activation_flex": max(h["a_flex"]),
        "max_activation_ext": max(h["a_ext"]),
        "max_torque": max(abs(x) for x in h["tau"]),
        "steps": len(t),
    }
