#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vehicle_model.py —— 一维纵向车辆运动学模型（用于 ACC 自适应巡航仿真）

模型描述：
    dv/dt = a，a 受加速度 / 制动减速度能力限制；
    dx/dt = v，v 限制在 [0, v_max]。
    控制器给出目标速度 v_cmd，模型按加速度限幅向目标速度收敛。

物理量单位：长度 m，速度 m/s，加速度 m/s^2，时间 s。
"""


class VehicleModel:
    """一维纵向车辆模型。"""

    def __init__(self, x=0.0, v=0.0, v_max=25.0,
                 a_acc=2.5, a_brk=4.0, dt=0.05):
        self.x = x                    # 纵向位置（m）
        self.v = v                    # 当前速度（m/s）
        self.v_max = v_max            # 最高车速（m/s）
        self.a_acc = a_acc            # 最大加速度（m/s^2）
        self.a_brk = a_brk            # 最大制动减速度（m/s^2，正数表示能力）
        self.dt = dt                  # 仿真步长（s）

    def step(self, v_cmd):
        """按加速度限幅向 v_cmd 逼近一个步长，并更新位置。

        v_cmd: 控制器给出的目标速度（m/s），会被限幅在 [0, v_max]。
        """
        v_cmd = min(max(v_cmd, 0.0), self.v_max)

        # 需要的加速度（带符号，正=加速 负=减速）
        need = (v_cmd - self.v) / self.dt
        a = min(max(need, -self.a_brk), self.a_acc)

        self.v = self.v + a * self.dt
        self.v = min(max(self.v, 0.0), self.v_max)
        self.x = self.x + self.v * self.dt
        return self.v


class LeadVehicle:
    """前车：按分段速度曲线行驶（巡航 -> 减速停车 -> 停止 -> 起步 -> 巡航）。"""

    def __init__(self, x=50.0, v=0.0, v_cruise=15.0,
                 a_acc=2.0, a_brk=3.0, dt=0.05):
        self.x = x
        self.v = v
        self.v_cruise = v_cruise
        self.a_acc = a_acc
        self.a_brk = a_brk
        self.dt = dt
        self.t = 0.0

    def set_phase_times(self, t_stop_begin, t_stop_end, t_go_begin):
        """设定场景时间节点：
        t_stop_begin  前车开始减速
        t_stop_end    前车完全停下
        t_go_begin    前车重新起步
        """
        self.t_stop_begin = t_stop_begin
        self.t_stop_end = t_stop_end
        self.t_go_begin = t_go_begin

    def step(self):
        """按时间推进前车速度（参考轨迹），返回当前速度。"""
        self.t += self.dt
        if self.t < self.t_stop_begin:
            # 巡航
            self._toward(self.v_cruise, self.a_acc)
        elif self.t < self.t_stop_end:
            # 减速停车
            self._toward(0.0, self.a_brk)
        elif self.t < self.t_go_begin:
            # 保持停止
            self._toward(0.0, self.a_acc)
        else:
            # 起步回到巡航
            self._toward(self.v_cruise, self.a_acc)
        self.x += self.v * self.dt
        return self.v

    def _toward(self, target, acc):
        dv = target - self.v
        if dv > 0:
            self.v += min(dv, self.a_acc * self.dt)
        elif dv < 0:
            self.v -= min(-dv, self.a_brk * self.dt)
        self.v = min(max(self.v, 0.0), self.v_cruise)
