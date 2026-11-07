#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Linear power/energy model for the delivery UAV.

单行英文摘要 + 中文正文（B 套风格）。
功率取线性近似：P(载重) = p_base_w + load_coef * m。两个参数的来源：
p_base_w = 40 W 是机架/航电/桨叶profile的空载基础功率；load_coef = 18 W/kg
是诱导功率随载重的线性增量（满载 2.5 kg 时 P = 85 W，对应小型物流四旋翼
悬停/巡航功率量级）。满载巡航续航 endurance_s = 1080 s（18 min），据此
反推电池容量 capacity_j = 85 W * 1080 s = 91800 J ≈ 25.5 Wh。
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class EnergyModel:
    """Battery + linear power model, all energy in joules / fractions."""
    p_base_w: float = 40.0
    load_coef_w_per_kg: float = 18.0
    endurance_ref_kg: float = 2.5
    endurance_s: float = 1080.0

    @property
    def capacity_j(self) -> float:
        return (self.p_base_w + self.load_coef_w_per_kg * self.endurance_ref_kg) * self.endurance_s

    def power_w(self, payload_kg: float) -> float:
        return self.p_base_w + self.load_coef_w_per_kg * max(0.0, payload_kg)

    def energy_j(self, payload_kg: float, duration_s: float) -> float:
        return self.power_w(payload_kg) * max(0.0, duration_s)

    def energy_frac(self, payload_kg: float, duration_s: float) -> float:
        """Battery fraction consumed over duration_s at given payload."""
        return self.energy_j(payload_kg, duration_s) / self.capacity_j

    def leg_frac(self, dist_m: float, payload_kg: float, speed_ms: float) -> float:
        """Battery fraction for one straight leg of dist_m."""
        if dist_m <= 0.0:
            return 0.0
        return self.energy_frac(payload_kg, dist_m / speed_ms)

    def hover_frac(self, payload_kg: float, hover_s: float) -> float:
        return self.energy_frac(payload_kg, hover_s)


#: 决策安全系数：剩余电量低于返航所需的 1.25 倍即触发换电
RETURN_MARGIN = 1.25


def needs_early_return(model: EnergyModel, battery_frac: float, dist_home_m: float,
                       payload_kg: float, speed_ms: float,
                       margin: float = RETURN_MARGIN) -> bool:
    """True if remaining battery is below margin x energy-to-get-home."""
    need = model.leg_frac(dist_home_m, payload_kg, speed_ms)
    return battery_frac < margin * need
