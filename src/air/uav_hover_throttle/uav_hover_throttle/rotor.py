"""螺旋桨推力/功率模型与悬停油门求解。

动量理论 + 简单叶素近似::

    推力     T = Ct * rho * n^2 * D^4            （n 为转速，rev/s）
    诱导功率 P_ind = T^1.5 / (FoM * sqrt(2 * rho * A_total))
    型阻功率 P_prof = k_prof * n^3
    电功率   P_elec = (P_ind + P_prof) / eta

悬停时 ``T = m*g``，因此

    n_hover = sqrt(m*g / (Ct * rho * D^4))

电机转速与电压近似成正比，``n_max(U) = n_nom * U / U_nom``，于是

    throttle_hover = n_hover / n_max(U)

空气密度按 ISA 标准大气随高度衰减。只用 Python 标准库。
"""

import math

G = 9.80665
RHO0 = 1.225                 # 海平面标准空气密度 kg/m^3
T0 = 288.15                  # 海平面标准温度 K
LAPSE = 0.0065               # 温度递减率 K/m


def air_density(altitude_m):
    """ISA 标准大气：``rho = rho0 * (1 - L*h/T0)^(g/(R*L) - 1)``。"""
    if altitude_m < -500.0 or altitude_m > 11000.0:
        raise ValueError('本模型只覆盖 -500 m ~ 11000 m')
    base = 1.0 - LAPSE * altitude_m / T0
    exponent = G / (287.05287 * LAPSE) - 1.0
    return RHO0 * base ** exponent


class Rotor:
    """一副螺旋桨的推力/功率特性。"""

    def __init__(self, diameter_m=0.254, thrust_coefficient=0.11,
                 figure_of_merit=0.75, profile_coefficient=8.0e-11,
                 nominal_voltage=16.8, nominal_rpm=9000.0):
        for name, value in (('diameter_m', diameter_m),
                            ('thrust_coefficient', thrust_coefficient),
                            ('figure_of_merit', figure_of_merit),
                            ('nominal_voltage', nominal_voltage),
                            ('nominal_rpm', nominal_rpm)):
            if value <= 0.0:
                raise ValueError(f'{name} 必须为正数')
        if figure_of_merit > 1.0:
            raise ValueError('figure_of_merit 不应大于 1')

        self.diameter = float(diameter_m)
        self.ct = float(thrust_coefficient)
        self.fom = float(figure_of_merit)
        self.k_prof = float(profile_coefficient)
        self.nominal_voltage = float(nominal_voltage)
        self.nominal_rpm = float(nominal_rpm)

    @property
    def disk_area(self):
        return math.pi * self.diameter ** 2 / 4.0

    # ---------- 正问题 ----------
    def thrust(self, rpm, altitude_m=0.0, rotors=4):
        """给定单桨转速求总推力（N）。"""
        if rpm < 0.0:
            raise ValueError('rpm 不能为负')
        n = rpm / 60.0
        rho = air_density(altitude_m)
        return rotors * self.ct * rho * n * n * self.diameter ** 4

    def electrical_power(self, rpm, altitude_m=0.0, rotors=4):
        """给定转速求总电功率（W）。"""
        if rpm <= 0.0:
            return 0.0
        n = rpm / 60.0
        rho = air_density(altitude_m)
        thrust = self.thrust(rpm, altitude_m, rotors)
        induced = thrust ** 1.5 / (
            self.fom * math.sqrt(2.0 * rho * rotors * self.disk_area))
        profile = rotors * self.k_prof * (rpm ** 3)
        return induced + profile

    # ---------- 反问题：悬停 ----------
    def hover_rpm(self, mass_kg, altitude_m=0.0, rotors=4):
        """悬停所需单桨转速（rpm）。"""
        if mass_kg <= 0.0:
            raise ValueError('质量必须为正数')
        rho = air_density(altitude_m)
        n = math.sqrt(mass_kg * G / (rotors * self.ct * rho * self.diameter ** 4))
        return n * 60.0

    def max_rpm(self, voltage):
        """电压对应的最大转速（线性近似）。"""
        if voltage <= 0.0:
            raise ValueError('电压必须为正数')
        return self.nominal_rpm * voltage / self.nominal_voltage

    def hover_throttle(self, mass_kg, voltage, altitude_m=0.0, rotors=4):
        """悬停油门，返回 ``(throttle, rpm_hover, rpm_max)``；超出 1 表示飞不起来。"""
        rpm_hover = self.hover_rpm(mass_kg, altitude_m, rotors)
        rpm_max = self.max_rpm(voltage)
        return rpm_hover / rpm_max, rpm_hover, rpm_max

    def hover_report(self, mass_kg, voltage, altitude_m=0.0, rotors=4):
        """一次性给出悬停的全部指标。"""
        throttle, rpm_hover, rpm_max = self.hover_throttle(
            mass_kg, voltage, altitude_m, rotors)
        power = self.electrical_power(rpm_hover, altitude_m, rotors)
        thrust = self.thrust(rpm_hover, altitude_m, rotors)
        return {
            'altitude_m': altitude_m,
            'voltage': voltage,
            'density': air_density(altitude_m),
            'rpm_hover': rpm_hover,
            'rpm_max': rpm_max,
            'throttle': throttle,
            'feasible': throttle <= 1.0,
            'thrust_n': thrust,
            'weight_n': mass_kg * G,
            'power_w': power,
            'power_per_kg_w': power / mass_kg,
        }
