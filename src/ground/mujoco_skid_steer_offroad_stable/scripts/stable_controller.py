"""车身姿态补偿控制器（实验性功能，默认不启用）。

作用方式：把俯仰角换算成前/后轮的速度差，前轮加速、后轮减速，试图在车身上产生
一个与俯仰方向相反的力矩。

实测结论（见 docs/ground/mujoco_skid_steer_offroad.md 第 8 节）：四轮为刚性底盘、
没有悬架，轮速差只能改变轮胎纵向滑移，俯仰角基本由地形几何决定，补偿带来的改善
在多个随机地形种子下并不稳定（-10% ~ +46%），因此主程序默认关闭该功能，仅在
``--stabilize`` 时启用，用于对照实验。
"""

import numpy as np


class AttitudeStableController:
    """俯仰角 P 补偿器，输出前/后轮差速补偿量（单位 m/s）。"""

    def __init__(self,
                 kp_pitch=10.0,
                 max_ratio=0.50,
                 deadband_deg=0.3):
        self.kp_pitch = kp_pitch
        self.max_ratio = max_ratio
        self.deadband = np.deg2rad(deadband_deg)

    def compute(self, pitch, base_speed):
        """返回补偿量 u（m/s）：前轮 +u、后轮 -u。

        pitch      : 俯仰角（rad）
        base_speed : 当前目标线速度（m/s），补偿量按比例限幅，避免低速时过度打滑
        """
        if abs(pitch) < self.deadband:
            pitch = 0.0
        else:
            pitch = pitch - np.sign(pitch) * self.deadband
        u = self.kp_pitch * pitch
        limit = self.max_ratio * max(base_speed, 0.1)
        return float(np.clip(u, -limit, limit))
