"""Stanley 控制器：作为 MPC 的对照基线。

Stanley 只看「当前」的横向误差与航向误差，没有预瞄：

    delta = -e_psi - atan(k * e_y / v) + atan(L * kappa_ref)

符号约定与 mpc.py 一致：e_y 以路径左法向为正（车在路径左侧时 e_y > 0），
e_psi = psi - psi_ref。因此修正项前面是负号——车偏左就要往右打方向。

为了让对比公平，转向限幅、转向速率限幅、纵向速度、参考路径、初始偏差
都与 MPC 那一侧完全相同。
"""

import math


class StanleyController(object):
    def __init__(self, wheelbase=2.5, k=1.5, delta_max=0.6, rate_max=1.6):
        self.L = float(wheelbase)
        self.k = float(k)
        self.delta_max = float(delta_max)
        self.rate_max = float(rate_max)     # rad/s，限制转向执行器速度
        self.delta_prev = 0.0

    def reset(self, delta0=0.0):
        self.delta_prev = float(delta0)

    def control(self, e_y, e_psi, kappa_ref, v, dt):
        if v < 0.05:
            v_eff = 0.05
        else:
            v_eff = v
        delta = -e_psi - math.atan(self.k * e_y / v_eff) \
                + math.atan(self.L * kappa_ref)
        delta = max(-self.delta_max, min(self.delta_max, delta))
        # 转向速率限幅
        limit = self.rate_max * dt
        delta = max(self.delta_prev - limit,
                    min(self.delta_prev + limit, delta))
        self.delta_prev = delta
        return delta
