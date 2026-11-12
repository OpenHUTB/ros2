"""多旋翼重心偏移配平与饱和包线核算（只依赖标准库）.

多旋翼的电池、相机或机械爪装偏之后，悬停时四个电机要出力不均来配平，重心偏得太多
电机先饱和，配平就做不到了。本模块算三件事：

1. 给定重心偏移，四只电机各要多少拉力（plus 构型：前后一对管俯仰，左右一对管横滚）；
2. 悬停时为了配平要倾斜多少度（推力线必须穿过重心）；
3. 包线：偏移大到什么程度电机饱和，还剩多少配平裕度。

符号与公式（plus 构型，前后电机力臂 b，左右电机力臂 a，重心在桨平面下方 h）：

    W = m*g
    T1 + T2 + T3 + T4 = W                      (悬停合力)
    (T1 - T3) * b = W * dx                     (俯仰力矩平衡)
    (T2 - T4) * a = W * dy                     (横滚力矩平衡)

    T1 = W/4 + W*dx/(2b),  T3 = W/4 - W*dx/(2b)
    T2 = W/4 + W*dy/(2a),  T4 = W/4 - W*dy/(2a)

    悬停倾角    theta = atan(dx / h),  phi = atan(dy / h)
    饱和包线    dx_max = 2b * (T_max/W - 1/4),  dy_max = 2a * (T_max/W - 1/4)

T_max/W < 1/4 时单电机带不动四分之一重量，任何偏移都无法悬停（判不可行）。
"""

import math
from dataclasses import dataclass

G = 9.80665


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _finite(value):
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def per_motor_thrusts(mass_kg, arm_x_m, arm_y_m, dx_m, dy_m, gravity=G):
    """返回 [前, 右, 后, 左] 四只电机的悬停拉力 N."""
    _require(_finite(mass_kg) and mass_kg > 0.0, '质量必须为正数')
    for name, value in (('arm_x_m', arm_x_m), ('arm_y_m', arm_y_m)):
        _require(_finite(value) and value > 0.0, '%s 必须为正数' % name)
    for name, value in (('dx_m', dx_m), ('dy_m', dy_m)):
        _require(_finite(value), '%s 必须是有限数' % name)
    weight = mass_kg * gravity
    quarter = weight / 4.0
    return [quarter + weight * dx_m / (2.0 * arm_y_m),   # 前（俯仰配平）
            quarter + weight * dy_m / (2.0 * arm_x_m),   # 右（横滚配平）
            quarter - weight * dx_m / (2.0 * arm_y_m),   # 后
            quarter - weight * dy_m / (2.0 * arm_x_m)]   # 左


def trim_tilt_deg(dx_m, dy_m, cog_depth_m):
    """配平悬停时的俯仰/横滚倾角 deg：tan = 偏移 / 桨平面到重心高度."""
    _require(_finite(dx_m) and _finite(dy_m), '偏移必须是有限数')
    _require(_finite(cog_depth_m) and cog_depth_m > 0.0, '重心深度必须为正数')
    return (math.degrees(math.atan2(dx_m, cog_depth_m)),
            math.degrees(math.atan2(dy_m, cog_depth_m)))


def saturation_envelope(mass_kg, arm_x_m, arm_y_m, thrust_max_n, gravity=G):
    """返回包线：能配平的最大纵向/横向偏移 m 与对应的饱和倾角 deg."""
    _require(_finite(mass_kg) and mass_kg > 0.0, '质量必须为正数')
    _require(_finite(thrust_max_n) and thrust_max_n > 0.0, '单电机最大拉力必须为正数')
    weight = mass_kg * gravity
    excess = thrust_max_n / weight - 0.25
    dx_max = 2.0 * arm_y_m * excess
    dy_max = 2.0 * arm_x_m * excess
    return {'dx_max_m': max(dx_max, 0.0), 'dy_max_m': max(dy_max, 0.0),
            'hover_feasible': excess > 0.0}


def margin_pct(offset_m, limit_m):
    """偏移离包线边界的相对裕度 %."""
    _require(_finite(offset_m) and _finite(limit_m), '偏移与限位必须是有限数')
    if limit_m <= 1e-12:
        return 0.0
    return max((limit_m - abs(offset_m)) / limit_m, 0.0) * 100.0


@dataclass
class UavCgTrim:
    """一次重心偏移配平核算."""

    mass_kg: float = 1.2
    arm_x_m: float = 0.25
    arm_y_m: float = 0.25
    dx_m: float = 0.003
    dy_m: float = 0.002
    cog_depth_m: float = 0.03
    thrust_max_n: float = 6.0

    def __post_init__(self):
        for name in ('mass_kg', 'arm_x_m', 'arm_y_m', 'dx_m', 'dy_m', 'cog_depth_m',
                     'thrust_max_n'):
            _require(_finite(getattr(self, name)), '%s 必须是有限数' % name)
        _require(self.mass_kg > 0.0, '质量必须为正数')
        _require(self.arm_x_m > 0.0 and self.arm_y_m > 0.0, '力臂必须为正数')
        _require(self.cog_depth_m > 0.0, '重心深度必须为正数')
        _require(self.thrust_max_n > 0.0, '单电机最大拉力必须为正数')

    def report(self):
        thrusts = per_motor_thrusts(self.mass_kg, self.arm_x_m, self.arm_y_m,
                                    self.dx_m, self.dy_m)
        weight = self.mass_kg * G
        theta, phi = trim_tilt_deg(self.dx_m, self.dy_m, self.cog_depth_m)
        envelope = saturation_envelope(self.mass_kg, self.arm_x_m, self.arm_y_m,
                                       self.thrust_max_n)
        tilt_sat = trim_tilt_deg(envelope['dx_max_m'], envelope['dy_max_m'],
                                 self.cog_depth_m)
        max_use = max(thrusts)
        level = 'ok'
        warnings = []
        if not envelope['hover_feasible']:
            level = 'over'
            warnings.append('单电机最大拉力不足四分之一重量，任何偏移都无法悬停')
        elif max_use > self.thrust_max_n:
            level = 'over'
            warnings.append('重心偏移超出包线，电机 %s 拉力 %.2f N 超过 %.2f N'
                            % ('前后' if thrusts[0] >= thrusts[2] else '后前',
                               max_use, self.thrust_max_n))
        elif max_use > 0.9 * self.thrust_max_n:
            level = 'marginal'
            warnings.append('最大电机拉力 %.2f N 已到上限的 90%%，配平裕度不足'
                            % max_use)
        if not warnings:
            warnings.append('配平可行，裕度 纵向 %.1f%% / 横向 %.1f%%'
                            % (margin_pct(self.dx_m, envelope['dx_max_m']),
                               margin_pct(self.dy_m, envelope['dy_max_m'])))
        return {
            'parameters': {
                'mass_kg': self.mass_kg,
                'arm_x_m': self.arm_x_m,
                'arm_y_m': self.arm_y_m,
                'dx_m': self.dx_m,
                'dy_m': self.dy_m,
                'cog_depth_m': self.cog_depth_m,
                'thrust_max_n': self.thrust_max_n,
            },
            'weight_n': weight,
            'motor_thrusts_n': {'front': thrusts[0], 'right': thrusts[1],
                                'rear': thrusts[2], 'left': thrusts[3]},
            'max_motor_thrust_n': max_use,
            'level': level,
            'trim_tilt_deg': {'pitch': theta, 'roll': phi},
            'envelope': {
                'dx_max_m': envelope['dx_max_m'],
                'dy_max_m': envelope['dy_max_m'],
                'hover_feasible': envelope['hover_feasible'],
                'sat_pitch_deg': tilt_sat[0],
                'sat_roll_deg': tilt_sat[1],
            },
            'margin_pct': {'pitch': margin_pct(self.dx_m, envelope['dx_max_m']),
                           'roll': margin_pct(self.dy_m, envelope['dy_max_m'])},
            'warnings': warnings,
            'notes': [
                'plus 构型的前后/左右通道解耦，公式只对这种构型严格成立',
                '悬停倾角按推力线穿过重心推导，忽略桨平面气动倾斜耦合',
                '未计入电池/负载随姿态的位移，偏移按刚体固定处理',
            ],
        }
