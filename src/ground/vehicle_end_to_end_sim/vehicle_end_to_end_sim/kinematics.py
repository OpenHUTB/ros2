r"""车辆运动学模型 —— 经典自行车模型（Kinematic Bicycle Model）。

    \dot{x} = v cos(θ)
    \dot{y} = v sin(θ)
    \dot{θ} = (v / L) tan(δ)

其中 L 为轴距（config.WHEELBASE_PIXEL），δ 为前轮转向角（由 steer 归一化后映射，
δ = steer * pi/4）。该模型保证转向轨迹平滑，符合阿克曼转向特性。
"""

import math

from . import config


class Car:
    """维护车辆在仿真坐标系中的物理状态（x, y, theta, v_pixel）。"""

    def __init__(self, x=config.START_X, y=config.START_Y, theta=config.START_THETA):
        self.x = x
        self.y = y
        self.theta = theta          # 航向角（rad）
        self.v_pixel = 0.0          # 底层仿真速度（像素/帧）
        self.steer = 0.0            # 前轮转向角（归一化到 [-1, 1]）

    def update(self):
        """按自行车运动学模型推进一帧。"""
        self.x += self.v_pixel * math.cos(self.theta)
        self.y += self.v_pixel * math.sin(self.theta)
        self.theta += (self.v_pixel / config.WHEELBASE_PIXEL) * math.tan(
            self.steer * config.MAX_STEER_RAD
        )
