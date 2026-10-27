#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
acc_controller.py —— ACC 自适应巡航控制律（恒定时距 + 平方根刹停曲线）

控制思想：
    期望车距  d_des = d0 + t_gap*v_host + t_react*max(v_host - v_lead, 0)
    其中 d0 为停车安全间距，t_gap 为跟车时距，t_react 为相对速度反应时间
    （前车比本车慢时额外加大期望车距，使本车提前减速）。

    三区控制：
      巡航区  d > d_enter            -> v_cmd = v_cruise（前车足够远，正常巡航）
      减速区  d0 < d <= d_enter      -> v_cmd = sqrt(2*a_brk*(d - d0))
                                     平方根刹停曲线：物理上保证本车在抵达
                                     d0 处速度恰好为 0，绝不越过前车停止点，
                                     且减速平滑（越接近前车速度越低）。
      停车区  d <= d0                -> v_cmd = 0（全力制动）

    停车锁定：前车已静止且本车低速接近时直接刹停，消除低速蠕动。

    控制器输出目标速度 v_cmd，由车辆模型按加速度能力平滑执行。
"""

import math


class ACCController:
    """恒定时距 + 平方根刹停曲线的 ACC 控制器。"""

    def __init__(self, v_cruise=15.0, d0=3.0, t_gap=1.0,
                 t_react=0.6, a_brk=4.0, margin=3.0, car_len=4.5):
        self.v_cruise = v_cruise   # 巡航速度（m/s）
        self.d0 = d0               # 停车安全间距（保险杠间距，m）
        self.t_gap = t_gap         # 跟车时距（s）
        self.t_react = t_react     # 相对速度反应时间（s）
        self.a_brk = a_brk         # 刹停曲线使用的最大制动减速度（m/s^2）
        self.margin = margin       # 巡航区 -> 减速区 的过渡余量（m）
        self.car_len = car_len     # 车长（m），中心距 -> 保险杠间距换算

    def bumper_gap(self, center_gap):
        """中心距 -> 保险杠间距：d_bumper = d_center - 车长。"""
        return center_gap - self.car_len

    def desired_gap(self, v_host, v_lead=0.0):
        """期望保险杠间距：d0 + t_gap*v_host + t_react*max(v_host-v_lead, 0)。"""
        rel = max(v_host - v_lead, 0.0)
        return self.d0 + self.t_gap * v_host + self.t_react * rel

    def compute(self, d_center, v_lead, v_host):
        """计算目标速度 v_cmd。

        参数:
            d_center: 两车中心距（m），d_center = x_lead - x_host
            v_lead:   前车速度（m/s）
            v_host:   本车速度（m/s）
        返回:
            (v_cmd, d_des_bumper)
        """
        d = self.bumper_gap(d_center)   # 控制律统一使用保险杠间距
        d_des = self.desired_gap(v_host, v_lead)
        d_enter = d_des + self.margin

        if d <= self.d0:
            # 停车区：全力制动
            v_cmd = 0.0
        elif d <= d_enter:
            # 减速区：平方根刹停曲线
            v_cmd = math.sqrt(2.0 * self.a_brk * max(d - self.d0, 0.0))
        else:
            # 巡航区
            v_cmd = self.v_cruise

        v_cmd = min(v_cmd, self.v_cruise)

        # 安全兜底：按最大制动减速度刹停所需距离超过可用空间时全力制动
        stop_need = (v_host * v_host - v_lead * v_lead) / (2.0 * self.a_brk)
        if stop_need > (d - self.d0):
            v_cmd = 0.0

        # 停车锁定：前车静止且本车低速贴近时直接刹停
        if v_lead < 0.1 and v_host < 0.5 and d <= self.d0 + 1.5:
            v_cmd = 0.0

        return v_cmd, d_des
