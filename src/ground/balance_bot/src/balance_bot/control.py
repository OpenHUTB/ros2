# -*- coding: utf-8 -*-
"""平衡控制器：倾角 PD + 速度环（LQR 的手工版）。

控制律：
    τ = -kp·θ - kd·θ̇ - kv·(v_target - v) + k_yaw·(ω_target - ω)

**速度项前面的负号是这次调试最关键的地方**：
    早期写成了 +kv·(v_target − v)，结果指令 +0.3 m/s 时车反而以 −3.7 m/s 倒着跑，
    静止时还会自己漂出 12 m。
    物理原因：自平衡车要往前进，必须先用**反向**轮矩把车身"压"成前倾（θ<0），
    车身前倾后轮子再追上去，整车才会向前。也就是说速度指令要先经过倾角，
    符号与直觉相反。改成负号后：v_target=0.3 → 实测 0.310 m/s，静止时不漂移。

θ 的符号约定：+ 表示车身**后仰**，− 表示前倾。
"""

from . import config


def wrap(a):
    import math
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class BalanceController(object):
    """倾角 PD + 速度环 + 转向环，输出左右轮力矩。"""

    def __init__(self, kp=None, kd=None, kv=None, k_yaw=None):
        self.kp = config.KP_PITCH if kp is None else kp
        self.kd = config.KD_PITCH if kd is None else kd
        self.kv = config.KV_SPEED if kv is None else kv
        self.k_yaw = config.K_YAW if k_yaw is None else k_yaw

    def __call__(self, st, v_target=0.0, w_target=0.0):
        """st：env.State；返回 (τ_left, τ_right)。"""
        tau = (-self.kp * st.theta
               - self.kd * st.theta_dot
               - self.kv * (v_target - st.v))
        tau_yaw = self.k_yaw * (w_target - st.w)
        # 差动：左轮 +τ，右轮 −τ 产生绕 z 的偏航力矩
        return tau - tau_yaw, tau + tau_yaw

    def gains(self):
        return dict(kp=self.kp, kd=self.kd, kv=self.kv, k_yaw=self.k_yaw)
