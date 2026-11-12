"""差速底盘驱动系统选型核算（只依赖标准库）.

给一台差速底盘配电机/减速箱时，先回答三个问题：

1. 轮边总共需要多大的牵引力（滚动 + 坡度 + 加速 + 风阻）；
2. 折算到每只驱动轮需要多大转矩、多少转速、多少功率；
3. 直接驱动行不行，不行的话减速比取多少、电机持续/峰值转矩够不够。

公式（全部可以手算核对）：

    F_roll  = f * m * g
    F_grade = m * g * sin(theta)
    F_accel = m * a * k_rot
    F_drag  = 0.5 * rho * Cd * A * v^2
    F_total = F_roll + F_grade + F_accel + F_drag(v_cruise)

    tau_wheel = F_total * r / n_driven / eta        (每只驱动轮转矩, N*m)
    rpm_wheel = v / (2*pi*r) * 60                  (巡航轮速, rpm)
    P_wheel   = F_total * v / n_driven             (每轮机械功率, W)
    P_input   = P_wheel / eta                      (每轮电功率, W)

    ratio     = rpm_motor_rated / rpm_wheel        (按电机额定转速配减速比)
    tau_motor = tau_wheel / (ratio * eta)          (折算到电机轴的需求转矩)

约定：差速底盘左右两轮独立驱动，n_driven=2；eta 同时计入减速箱与传动效率。
"""

import math
from dataclasses import dataclass

G = 9.80665
RHO_AIR = 1.225


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _finite(value):
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def rolling_force_n(mass_kg, rolling_coef, gravity=G):
    _require(_finite(mass_kg) and mass_kg > 0.0, '质量必须为正数')
    _require(_finite(rolling_coef) and rolling_coef >= 0.0, '滚动阻力系数不能为负')
    return rolling_coef * mass_kg * gravity


def grade_force_n(mass_kg, grade_deg, gravity=G):
    _require(_finite(mass_kg) and mass_kg > 0.0, '质量必须为正数')
    _require(_finite(grade_deg) and -45.0 <= grade_deg <= 45.0, '坡度需在 +-45 deg 内')
    return mass_kg * gravity * math.sin(math.radians(grade_deg))


def accel_force_n(mass_kg, accel_m_s2, spin_factor):
    _require(_finite(mass_kg) and mass_kg > 0.0, '质量必须为正数')
    _require(_finite(accel_m_s2) and accel_m_s2 >= 0.0, '加速度不能为负')
    _require(_finite(spin_factor) and spin_factor >= 1.0, '旋转惯量系数需不小于 1')
    return mass_kg * accel_m_s2 * spin_factor


def drag_force_n(velocity_m_s, drag_coef, front_area_m2, rho=RHO_AIR):
    _require(_finite(velocity_m_s) and velocity_m_s >= 0.0, '速度不能为负')
    _require(_finite(drag_coef) and drag_coef >= 0.0, '风阻系数不能为负')
    _require(_finite(front_area_m2) and front_area_m2 >= 0.0, '迎风面积不能为负')
    _require(_finite(rho) and rho > 0.0, '空气密度必须为正数')
    return 0.5 * rho * drag_coef * front_area_m2 * velocity_m_s ** 2


def wheel_rpm(velocity_m_s, wheel_radius_m):
    _require(_finite(velocity_m_s) and velocity_m_s >= 0.0, '速度不能为负')
    _require(_finite(wheel_radius_m) and wheel_radius_m > 0.0, '轮半径必须为正数')
    return velocity_m_s / (2.0 * math.pi * wheel_radius_m) * 60.0


def torque_level(torque_n_m, cont_n_m, peak_n_m):
    """需求转矩对电机能力的三档判定：ok / marginal / over."""
    _require(_finite(torque_n_m) and torque_n_m >= 0.0, '需求转矩不能为负')
    _require(_finite(cont_n_m) and cont_n_m > 0.0, '持续转矩必须为正数')
    _require(_finite(peak_n_m) and peak_n_m >= cont_n_m, '峰值转矩需不小于持续转矩')
    if torque_n_m <= cont_n_m:
        return 'ok'
    if torque_n_m <= peak_n_m:
        return 'marginal'
    return 'over'


@dataclass
class PowertrainCheck:
    """一次完整的驱动选型核算."""

    mass_kg: float = 12.0
    wheel_radius_m: float = 0.06
    driven_wheels: int = 2
    rolling_coef: float = 0.03
    grade_deg: float = 10.0
    accel_m_s2: float = 1.0
    v_cruise_m_s: float = 1.5
    drag_coef: float = 0.8
    front_area_m2: float = 0.06
    drive_efficiency: float = 0.85
    spin_factor: float = 1.08
    motor_cont_torque_nm: float = 1.2
    motor_peak_torque_nm: float = 2.5
    motor_rated_rpm: float = 4000.0
    bus_voltage_v: float = 24.0

    def __post_init__(self):
        for name in ('mass_kg', 'wheel_radius_m', 'rolling_coef', 'accel_m_s2',
                     'v_cruise_m_s', 'drag_coef', 'front_area_m2', 'drive_efficiency',
                     'spin_factor', 'motor_cont_torque_nm', 'motor_peak_torque_nm',
                     'motor_rated_rpm', 'bus_voltage_v'):
            _require(_finite(getattr(self, name)), '%s 必须是有限数' % name)
        _require(self.mass_kg > 0.0, '质量必须为正数')
        _require(self.wheel_radius_m > 0.0, '轮半径必须为正数')
        _require(isinstance(self.driven_wheels, int) and self.driven_wheels >= 1,
                 '驱动轮数量需为正整数')
        _require(0.0 < self.drive_efficiency <= 1.0, '传动效率需在 (0, 1] 内')
        _require(self.spin_factor >= 1.0, '旋转惯量系数需不小于 1')
        _require(self.v_cruise_m_s > 0.0, '巡航速度必须为正数')
        _require(self.bus_voltage_v > 0.0, '母线电压必须为正数')
        _require(self.motor_peak_torque_nm >= self.motor_cont_torque_nm,
                 '峰值转矩需不小于持续转矩')
        _require(0.0 <= self.grade_deg <= 45.0, '坡度需在 0~45 deg 内')

    # ------------------------------------------------------------------ 各阻力
    def forces(self):
        return {
            'rolling_n': rolling_force_n(self.mass_kg, self.rolling_coef),
            'grade_n': grade_force_n(self.mass_kg, self.grade_deg),
            'accel_n': accel_force_n(self.mass_kg, self.accel_m_s2, self.spin_factor),
            'drag_n': drag_force_n(self.v_cruise_m_s, self.drag_coef, self.front_area_m2),
        }

    def total_force_n(self):
        return sum(self.forces().values())

    # ------------------------------------------------------------------ 轮端
    def wheel_quantities(self):
        force = self.total_force_n()
        tau = force * self.wheel_radius_m / self.driven_wheels / self.drive_efficiency
        rpm = wheel_rpm(self.v_cruise_m_s, self.wheel_radius_m)
        power = force * self.v_cruise_m_s / self.driven_wheels / self.drive_efficiency
        return {'torque_nm': tau, 'rpm': rpm, 'power_w': power}

    # ------------------------------------------------------------------ 电机
    def motor_match(self):
        wheel = self.wheel_quantities()
        ratio = self.motor_rated_rpm / wheel['rpm']
        torque_geared = wheel['torque_nm'] / (ratio * self.drive_efficiency)
        torque_direct = wheel['torque_nm'] / self.drive_efficiency
        current = wheel['power_w'] / self.bus_voltage_v
        return {
            'gear_ratio': ratio,
            'torque_direct_nm': torque_direct,
            'torque_direct_level': torque_level(torque_direct, self.motor_cont_torque_nm,
                                                self.motor_peak_torque_nm),
            'torque_geared_nm': torque_geared,
            'torque_geared_level': torque_level(torque_geared, self.motor_cont_torque_nm,
                                                self.motor_peak_torque_nm),
            'motor_rpm_at_cruise': self.motor_rated_rpm,
            'current_per_motor_a': current,
        }

    def report(self):
        forces = self.forces()
        wheel = self.wheel_quantities()
        motor = self.motor_match()
        warnings = []
        if motor['torque_direct_level'] == 'over':
            warnings.append('直接驱动需求转矩超过电机峰值，必须加减速箱')
        elif motor['torque_direct_level'] == 'marginal':
            warnings.append('直接驱动需求转矩超过持续转矩（短时可过载），建议按 %.2f 配减速箱'
                            % motor['gear_ratio'])
        if motor['torque_geared_level'] == 'marginal':
            warnings.append('减速后需求转矩仍超过电机持续转矩，换大电机或更大减速比')
        if not warnings:
            warnings.append('按额定转速配 %.2f 减速比后，需求转矩在电机持续能力内'
                            % motor['gear_ratio'])
        return {
            'parameters': {
                'mass_kg': self.mass_kg,
                'wheel_radius_m': self.wheel_radius_m,
                'driven_wheels': self.driven_wheels,
                'rolling_coef': self.rolling_coef,
                'grade_deg': self.grade_deg,
                'accel_m_s2': self.accel_m_s2,
                'v_cruise_m_s': self.v_cruise_m_s,
                'drag_coef': self.drag_coef,
                'front_area_m2': self.front_area_m2,
                'drive_efficiency': self.drive_efficiency,
                'spin_factor': self.spin_factor,
                'motor_cont_torque_nm': self.motor_cont_torque_nm,
                'motor_peak_torque_nm': self.motor_peak_torque_nm,
                'motor_rated_rpm': self.motor_rated_rpm,
                'bus_voltage_v': self.bus_voltage_v,
            },
            'forces': forces,
            'total_force_n': self.total_force_n(),
            'wheel': wheel,
            'motor': motor,
            'warnings': warnings,
            'notes': [
                'eta 同时计入减速箱与传动效率，未单独拆开',
                '按上坡+匀加速+巡航风阻同时取满的保守工况核算',
                '不含轮胎打滑限制；地面附着力不足时先按 mu*m*g 校核再取小值',
            ],
        }
