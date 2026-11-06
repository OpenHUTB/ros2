"""红绿灯启停控制器。

规则：
- 绿灯：以巡航速度行驶；
- 红灯/黄灯：按"平方根减速度曲线"规划目标速度，
  保证车辆在停止线前的安全距离内平滑停住，不越线。

实现要点：
1. 距离按"车头"计算（head = x + 车长），停止线约束的是车头；
2. 目标速度 v_max = sqrt(2 * a_max * d_remain)（d_remain 为车头剩余可行驶距离）；
3. 当刹停距离 v^2/(2*a_max) 接近剩余距离时，直接全力刹车兜底，
   消除一阶追踪滞后带来的越线风险。
"""
import math


class StopController:
    def __init__(self, v_cruise=10.0, stop_x=50.0, safety=1.0,
                 a_max=3.0, a_acc=2.0, tau=0.5, brake_ratio=0.95):
        self.v_cruise = float(v_cruise)   # 巡航速度 m/s
        self.stop_x = float(stop_x)       # 停止线位置 m
        self.safety = float(safety)       # 停车时车头距停止线的安全余量 m
        self.a_max = float(a_max)         # 最大减速度 m/s^2
        self.a_acc = float(a_acc)         # 最大加速度 m/s^2
        self.tau = float(tau)             # 一阶趋近时间常数 s
        self.brake_ratio = float(brake_ratio)  # 全力刹车触发阈值（比例）

    def target_speed(self, x_head, light_color):
        """根据灯色与车头位置计算目标速度。"""
        if light_color == 'green':
            return self.v_cruise
        d = (self.stop_x - self.safety) - x_head
        if d <= 0.0:
            return 0.0
        return min(self.v_cruise, math.sqrt(2.0 * self.a_max * d))

    def compute(self, x_head, v, light_color):
        """返回加速度指令（m/s^2）。x_head 为车头位置。"""
        if light_color == 'green':
            v_target = self.v_cruise
        else:
            d = (self.stop_x - self.safety) - x_head
            if d <= 0.0 or (v * v) / (2.0 * self.a_max) >= d * self.brake_ratio:
                return -self.a_max          # 兜底：全力刹车
            v_target = min(self.v_cruise,
                           math.sqrt(2.0 * self.a_max * d))
        a = (v_target - v) / self.tau
        return max(-self.a_max, min(self.a_acc, a))
