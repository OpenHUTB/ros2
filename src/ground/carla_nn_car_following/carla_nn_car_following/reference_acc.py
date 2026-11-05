# -*- coding: utf-8 -*-
"""经典 ACC 控制器 —— 神经网络的训练标签来源。

本文件**不依赖 carla、也不依赖 numpy**，因此可以在任意 Python 环境中单独运行
单元测试（见 ``test/test_reference_acc.py``）。所有公式与文档第 3.2 节一一对应。

符号约定
--------
* ``d``      : 实际车距（m），自车雷达安装点到前车参考点
* ``v_ego``  : 自车速度（m/s）
* ``v_rel``  : 相对速度 ``v_lead - v_ego``（m/s），**负值表示正在接近前车**
* ``a_cmd``  : 期望加速度（m/s^2），正值加速、负值减速
"""

EPS = 1e-6


class ReferenceACC:
    """固定车头时距（CTH）ACC 控制器。"""

    def __init__(self, d0=5.0, headway=1.2, kp=0.35, kv=1.5,
                 k_throttle=0.25, k_brake=0.35, v_set=None, k_speed=1.0,
                 k_feedforward=0.03):
        self.d0 = float(d0)                 # 静止安全距离（m）
        self.headway = float(headway)       # 车头时距（s）
        self.kp = float(kp)                 # 车距误差增益
        self.kv = float(kv)                 # 相对速度增益
        self.k_throttle = float(k_throttle)  # 加速度 -> 油门
        self.k_brake = float(k_brake)        # 减速度 -> 刹车
        self.v_set = None if v_set in (None, '') else float(v_set)  # 设定车速（m/s）
        self.k_speed = float(k_speed)        # 设定车速限制的增益
        self.k_feedforward = float(k_feedforward)   # 前馈项（抵消滚阻/风阻）

    # ---- 公式(1)：期望车距 d_des = d0 + t_h * v_ego ----
    def desired_gap(self, v_ego):
        return self.d0 + self.headway * v_ego

    # ---- 公式(2)：车距误差 e = d - d_des ----
    def gap_error(self, d, v_ego):
        return d - self.desired_gap(v_ego)

    # ---- 公式(3)：a_cmd = kp * e + kv * v_rel，并受设定车速限制 ----
    def acceleration(self, d, v_rel, v_ego):
        a_gap = self.kp * self.gap_error(d, v_ego) + self.kv * v_rel
        if self.v_set is None:
            return a_gap
        # 设定车速限制：前车很远或跟丢时，车速不会超过 v_set。
        # 没有这一项时，一旦前车驶出本车道，自车会为了"追上车距"而无限加速。
        return min(a_gap, self.k_speed * (self.v_set - v_ego))

    # ---- 公式(4)：加速度映射为油门/刹车（两者互斥）----
    def labels(self, d, v_rel, v_ego):
        """返回 ``(throttle, brake)``，取值范围均为 [0, 1]。

        油门上叠加了一个**前馈项** ``k_ff * v_e``，用于抵消滚动阻力与空气阻力。
        没有它时，为了维持巡航速度必须长期靠"车距误差"去换油门，
        会出现 5 m 左右的稳态误差（实测车距从 25 m 缓慢漂到 30 m）。
        """
        a_cmd = self.acceleration(d, v_rel, v_ego)
        if a_cmd >= 0.0:
            throttle = self.k_throttle * a_cmd + self.k_feedforward * v_ego
            return min(1.0, max(0.0, throttle)), 0.0
        return 0.0, min(1.0, self.k_brake * (-a_cmd))

    # ---- 公式(5)：TTC = d / (v_ego - v_lead) = d / (-v_rel) ----
    @staticmethod
    def ttc(d, v_rel):
        """碰撞时间。不在接近时返回 ``inf``。"""
        closing = -v_rel
        if closing <= EPS:
            return float('inf')
        return d / closing

    # ---- 跟车状态分类（感知网络的分类头标签）----
    def follow_state(self, d, v_ego):
        """0 = 远，1 = 近，2 = 危险。"""
        error = self.gap_error(d, v_ego)
        if error < -2.0:
            return 2
        if error < 3.0:
            return 1
        return 0
