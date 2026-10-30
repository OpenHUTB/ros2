"""车辆运动学模型：纵向（位移/速度/加速度）+ 横向（车道偏移）。"""

import math


class VehicleModel:
    """简化运动学模型：纵向一维 + 横向一维。

    纵向 x 沿道路方向，横向 y 为车道偏移（0 = 原车道中心线）。
    忽略转向动力学细节，用纵向加速度 a 与横向速度 vy 直接驱动。
    """

    def __init__(self, x=0.0, y=0.0, v=0.0, length=4.5, width=1.8):
        self.x = x            # 纵向位置（m）
        self.y = y            # 横向位置（m）
        self.v = v            # 纵向速度（m/s）
        self.yaw = 0.0        # 航向角（rad，仅用于绘制）
        self.length = length  # 车长（m）
        self.width = width    # 车宽（m）
        self.max_v = 25.0     # 最大车速（m/s）

    def update(self, a, vy, dt):
        """a: 纵向加速度 m/s^2；vy: 横向速度 m/s；dt: 步长 s。"""
        self.v = max(0.0, min(self.max_v, self.v + a * dt))
        self.x += self.v * dt
        self.y += vy * dt
        if self.v > 0.5:
            self.yaw = math.atan2(vy, self.v)
        else:
            self.yaw = 0.0
