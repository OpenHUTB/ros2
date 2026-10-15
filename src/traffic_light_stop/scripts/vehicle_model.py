"""一维车辆运动学模型（沿道路直线行驶）。

状态：x（沿道路位置，单位 m）、v（速度，单位 m/s）。
输入：加速度指令 accel（m/s^2）。
"""
class VehicleModel:
    def __init__(self, x=0.0, v=0.0, length=4.4, dt=0.05):
        self.x = float(x)
        self.v = float(v)
        self.length = float(length)   # 车长，用于计算车头位置
        self.dt = float(dt)

    def head(self):
        """车头位置（车辆沿 x 正向行驶，车头在最前方）。"""
        return self.x + self.length

    def step(self, accel):
        """按给定加速度推进一个仿真步长。速度不允许为负（不倒车）。"""
        v_new = self.v + accel * self.dt
        if v_new < 0.0:
            v_new = 0.0
        self.x += (self.v + v_new) * 0.5 * self.dt
        self.v = v_new
