# -*- coding: utf-8 -*-
"""坡道起步辅助：四轮轮角保持 PD + 坡度前馈。

    τ_i = [k_p·(φ_i0 − φ_i) − k_d·φ̇_i + (m·g·sinα·r)/4]

前馈项按 4 个轮平摊，抵消重力沿坡分量；PD 把轮子锁在落稳后的角度上。
只给 PD 会有稳态残差（要靠误差才出力矩），车会缓慢蠕动。
"""

import math

from . import config


class HillStartController(object):

    def __init__(self, kp=None, kd=None, slope_deg=None, mass=None):
        self.kp = config.KP_HOLD if kp is None else kp
        self.kd = config.KD_HOLD if kd is None else kd
        self.slope = config.SLOPE_DEG if slope_deg is None else slope_deg
        # 质量优先由调用方从 model.body_mass.sum() 传入，避免与场景不同步
        self.mass = config.MASS if mass is None else mass

    def feedforward_per_wheel(self):
        total = (self.mass * config.GRAVITY
                 * math.sin(math.radians(self.slope)) * config.WHEEL_RADIUS)
        return total / 4.0

    def torque(self, phi_err, phi_dot):
        tau = -self.kp * phi_err - self.kd * phi_dot + self.feedforward_per_wheel()
        return max(-config.TAU_MAX, min(config.TAU_MAX, tau))

    def gains(self):
        return dict(kp=self.kp, kd=self.kd, slope_deg=self.slope,
                    mass=round(self.mass, 3),
                    ff_per_wheel=round(self.feedforward_per_wheel(), 4))
