"""USV 弗劳德缩比换算与拖曳功率估算（只依赖标准库）.

船模试验的整套换算逻辑：

1. **弗劳德相似**：兴波现象由 Fr = v/sqrt(g*L) 决定。要让船模和实船的波系相似，
   模型速度取 v_m = v_s / sqrt(lambda)，lambda 是缩尺比（实船长 / 模型长）。

2. **力的换算**：水动力按 lambda^3 放大（同密度水），功率按 lambda^3.5 放大：
       R_s = R_m * lambda^3
       P_s = P_m * lambda^3.5

3. **ITTC 1957 分解法**：总阻力 = 摩擦阻力 + 剩余阻力。摩擦阻力用平板摩擦公式算，
   剩余阻力（兴波+形状）按弗劳德律放大：
       C_f    = 0.075 / (log10(Re) - 2)^2,   Re = v*L/nu
       R_f    = 0.5 * rho * S * v^2 * C_f
       R_r_m  = R_total_m - R_f_m
       R_s    = R_f_s + R_r_m * lambda^3
       P_e    = R_s * v_s                    (有效功率)
       P_d    = P_e / eta                    (螺旋桨轴功率, eta 为准推进效率)

符号：S 为湿表面积（模型取 S_s/lambda^2），nu 为运动黏度。
"""

import math
from dataclasses import dataclass

G = 9.80665
RHO_SEA = 1025.0
NU_SEA = 1.19e-6


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _finite(value):
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def froude_number(speed_m_s, length_m):
    _require(_finite(speed_m_s) and speed_m_s > 0.0, '速度必须为正数')
    _require(_finite(length_m) and length_m > 0.0, '船长必须为正数')
    return speed_m_s / math.sqrt(G * length_m)


def model_speed_for_froude(ship_speed_m_s, scale):
    """同 Fr 下的模型速度 v_m = v_s / sqrt(lambda)."""
    _require(_finite(ship_speed_m_s) and ship_speed_m_s > 0.0, '实船速度必须为正数')
    _require(_finite(scale) and scale > 1.0, '缩尺比必须大于 1')
    return ship_speed_m_s / math.sqrt(scale)


def scale_resistance(resistance_model_n, scale):
    """阻力按 lambda^3 放大."""
    _require(_finite(resistance_model_n) and resistance_model_n >= 0.0, '模型阻力不能为负')
    _require(_finite(scale) and scale > 1.0, '缩尺比必须大于 1')
    return resistance_model_n * scale ** 3


def scale_power(power_model_w, scale):
    """功率按 lambda^3.5 放大."""
    _require(_finite(power_model_w) and power_model_w >= 0.0, '模型功率不能为负')
    _require(_finite(scale) and scale > 1.0, '缩尺比必须大于 1')
    return power_model_w * scale ** 3.5


def friction_coef_itfc(reynolds):
    """ITTC 1957 摩擦阻力系数（平板相关线）."""
    _require(_finite(reynolds) and reynolds > 1.0, '雷诺数必须大于 1')
    return 0.075 / (math.log10(reynolds) - 2.0) ** 2


def friction_resistance(speed_m_s, length_m, wetted_area_m2, rho=RHO_SEA,
                        nu=NU_SEA):
    """平板摩擦阻力 N."""
    _require(_finite(speed_m_s) and speed_m_s > 0.0, '速度必须为正数')
    _require(_finite(length_m) and length_m > 0.0, '船长必须为正数')
    _require(_finite(wetted_area_m2) and wetted_area_m2 > 0.0, '湿表面积必须为正数')
    _require(_finite(rho) and rho > 0.0, '密度必须为正数')
    _require(_finite(nu) and nu > 0.0, '运动黏度必须为正数')
    reynolds = speed_m_s * length_m / nu
    return 0.5 * rho * wetted_area_m2 * speed_m_s ** 2 * friction_coef_itfc(reynolds)


@dataclass
class UsvFroude:
    """一次缩比换算：船模实测总阻力 -> 实船总阻力与拖曳功率."""

    ship_speed_m_s: float = 5.0
    ship_length_m: float = 10.0
    scale: float = 10.0
    wetted_area_ship_m2: float = 22.0
    model_resistance_n: float = 2.5
    propulsive_efficiency: float = 0.55
    rho: float = RHO_SEA
    nu: float = NU_SEA

    def __post_init__(self):
        for name in ('ship_speed_m_s', 'ship_length_m', 'scale', 'wetted_area_ship_m2',
                     'model_resistance_n', 'propulsive_efficiency', 'rho', 'nu'):
            _require(_finite(getattr(self, name)), '%s 必须是有限数' % name)
        _require(self.ship_speed_m_s > 0.0, '实船速度必须为正数')
        _require(self.ship_length_m > 0.0, '船长必须为正数')
        _require(self.scale > 1.0, '缩尺比必须大于 1')
        _require(self.wetted_area_ship_m2 > 0.0, '湿表面积必须为正数')
        _require(self.model_resistance_n >= 0.0, '模型阻力不能为负')
        _require(0.0 < self.propulsive_efficiency <= 1.0, '推进效率需在 (0, 1] 内')
        _require(self.rho > 0.0 and self.nu > 0.0, '密度与黏度必须为正数')

    def report(self):
        length_model = self.ship_length_m / self.scale
        speed_model = model_speed_for_froude(self.ship_speed_m_s, self.scale)
        area_model = self.wetted_area_ship_m2 / self.scale ** 2
        fr = froude_number(self.ship_speed_m_s, self.ship_length_m)

        re_ship = self.ship_speed_m_s * self.ship_length_m / self.nu
        re_model = speed_model * length_model / self.nu
        cf_ship = friction_coef_itfc(re_ship)
        cf_model = friction_coef_itfc(re_model)
        rf_ship = friction_resistance(self.ship_speed_m_s, self.ship_length_m,
                                      self.wetted_area_ship_m2, self.rho, self.nu)
        rf_model = friction_resistance(speed_model, length_model, area_model,
                                       self.rho, self.nu)
        rr_model = self.model_resistance_n - rf_model
        total_ship = rf_ship + rr_model * self.scale ** 3
        power_effective = total_ship * self.ship_speed_m_s
        power_delivered = power_effective / self.propulsive_efficiency

        warnings = []
        if rr_model <= 0.0:
            warnings.append('模型剩余阻力不大于零，说明实测总阻力小于平板摩擦估算，'
                            '检查模型阻力测量或湿表面积')
        if fr > 1.0:
            warnings.append('Fr=%.2f 超过 1，兴波阻力规律变化，弗劳德换算不再可靠'
                            % fr)
        if not warnings:
            warnings.append('Fr=%.3f，剩余阻力占实船总阻力 %.0f%%'
                            % (fr, rr_model * self.scale ** 3 / total_ship * 100.0))
        return {
            'parameters': {
                'ship_speed_m_s': self.ship_speed_m_s,
                'ship_length_m': self.ship_length_m,
                'scale': self.scale,
                'wetted_area_ship_m2': self.wetted_area_ship_m2,
                'model_resistance_n': self.model_resistance_n,
                'propulsive_efficiency': self.propulsive_efficiency,
                'rho': self.rho,
                'nu': self.nu,
            },
            'model': {
                'length_m': length_model,
                'speed_m_s': speed_model,
                'wetted_area_m2': area_model,
                'friction_resistance_n': rf_model,
                'residuary_resistance_n': rr_model,
            },
            'ship': {
                'froude_number': fr,
                'reynolds': re_ship,
                'friction_coef': cf_ship,
                'friction_resistance_n': rf_ship,
                'total_resistance_n': total_ship,
            },
            'power': {
                'effective_w': power_effective,
                'delivered_w': power_delivered,
            },
            'warnings': warnings,
            'notes': [
                'ITTC 1957 平板相关线 + 弗劳德换算剩余阻力，粘性尺度效应用形况系数修正（本模块取 1）',
                '船模试验数据的换算链：实测总阻力 -> 减摩擦 -> 乘 lambda^3 -> 加实船摩擦',
                '未计入空气阻力、附体阻力与粗糙度补贴，工程上通常再加 5%~15% 裕量',
            ],
        }
