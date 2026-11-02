#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
battery_monitor_rth/core.py - 锂电池放电模型与低电量自动返航（RTH）触发核心

模型（简化二阶 RC 等效的工程近似）:
    - 开路电压 OVC(SOC): 锂电池典型放电曲线分段线性近似 (4.2V -> 3.3V)
    - 负载压降: I_load * R_int（内阻 0.05Ω）
    - SOC 由电流积分消耗: dSOC = -I * dt / 容量(安时)
返航触发策略（三级告警）:
    - SOC <= 50%: 注意 (INFO)
    - SOC <= 25%: 返航 (RTH)
    - SOC <= 10%: 强制降落 (LAND)
"""

import math

CELLS = 4          # 4S 电池
CAPACITY_AH = 5.0  # 5000mAh
R_INTERNAL = 0.05  # 内阻 Ω


def open_circuit_voltage(soc: float) -> float:
    """单节锂电池开路电压 (V), SOC ∈ [0,1]。"""
    soc = max(0.0, min(1.0, soc))
    # 分段线性: 100%->4.18, 80%->4.0, 50%->3.85, 20%->3.6, 0%->3.27
    pts = [(1.0, 4.18), (0.8, 4.00), (0.5, 3.85), (0.2, 3.60), (0.0, 3.27)]
    for (s1, v1), (s2, v2) in zip(pts, pts[1:]):
        if soc <= s1 and soc >= s2:
            return v2 + (soc - s2) / (s1 - s2) * (v1 - v2)
    return 3.27


def pack_voltage(soc: float, current: float) -> float:
    """整包端电压 = CELLS * (OCV - I*R_int)。"""
    return CELLS * (open_circuit_voltage(soc) - current * R_INTERNAL)


class BatterySim:
    """电池放电仿真器。"""

    def __init__(self, soc0=1.0, capacity_ah=CAPACITY_AH):
        self.soc = soc0
        self.capacity_ah = capacity_ah

    def step(self, current: float, dt: float) -> float:
        """施加负载电流 (A) dt 秒, 返回新 SOC。"""
        self.soc = max(0.0, self.soc - current * dt / 3600.0 / self.capacity_ah)
        return self.soc

    def voltage(self, current: float) -> float:
        return pack_voltage(self.soc, current)


class RthMonitor:
    """低电量分级告警 / 返航触发器。"""

    LEVELS = [('LAND', 0.10), ('RTH', 0.25), ('INFO', 0.50)]

    def __init__(self):
        self.triggered = set()

    def check(self, soc: float):
        """返回当前应触发的最高优先级动作 (未触发过则返回 None)。"""
        for name, threshold in self.LEVELS:  # 优先级: LAND > RTH > INFO
            if soc <= threshold and name not in self.triggered:
                self.triggered.add(name)
                return name
        return None
