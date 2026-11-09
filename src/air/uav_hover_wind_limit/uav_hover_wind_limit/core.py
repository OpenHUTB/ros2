"""悬停抗风能力核算: 倾角、推重比余量与蒲福风级.

模型是准静态的: 悬停时气流速度等于风速, 机体为刚性体, 气动合力只有阻力
D = 0.5 * rho * (Cd*A) * V^2, 方向与来流一致; 忽略旋翼入流对机体的遮挡与
机体升力、忽略姿态角对 Cd*A 的影响. 推力 T 沿机体 z 轴, 于是

    T * cos(theta) = m * g          （垂直方向平衡重量）
    T * sin(theta) = D(V)           （水平方向平衡阻力）

两式相除得到

    tan(theta) = 0.5 * rho * (Cd*A) * V^2 / (m * g)

由此可以在给定最大倾角 theta_max 时反解出能悬停的最大风速, 也可以算悬停所需
推重比 T/W = 1 / cos(theta_max). 结论用于选型与作业窗口评估, 不代替带气动
干扰的整机仿真或试飞.
"""
import bisect
import math

RHO0 = 1.225          # 海平面标准大气密度 kg/m^3
G0 = 9.80665          # 标准重力加速度 m/s^2
TROPOPAUSE_EXPONENT = 4.25588

# 蒲福风级上界 (m/s), 按 WMO 划分: 0 级 <0.3, 1 级 0.3~1.5, ...
BEAUFORT_UPPER_BOUNDS = (
    0.3, 1.6, 3.4, 5.5, 8.0, 10.8, 13.9, 17.2, 20.8, 24.5, 28.5, 32.7,
)


def air_density(altitude_m=0.0):
    """ISA 对流层密度近似: rho = rho0 * (1 - 2.25577e-5 * h) ** 4.25588.

    只在对流层内（约 -0.5 ~ 11 km）有效，超出范围直接报错，避免给出看似合理的错值。
    """
    if not math.isfinite(altitude_m):
        raise ValueError('高度必须是有限数值')
    if not -500.0 <= altitude_m <= 11000.0:
        raise ValueError('ISA 对流层近似只适用于 -500..11000 m')
    factor = 1.0 - 2.25577e-5 * altitude_m
    return RHO0 * factor ** TROPOPAUSE_EXPONENT


def beaufort_level(speed_m_s):
    """把风速 (m/s) 换算成蒲福风级 0..12."""
    if not math.isfinite(speed_m_s) or speed_m_s < 0.0:
        raise ValueError('风速必须是有限的非负数')
    return bisect.bisect_right(BEAUFORT_UPPER_BOUNDS, speed_m_s)


class HoverWindLimit:
    """给定质量、阻力面积与最大倾角时的悬停抗风核算."""

    def __init__(self, mass=2.0, drag_area=0.15, max_tilt_deg=25.0,
                 gravity=G0, rho=RHO0):
        values = (mass, drag_area, max_tilt_deg, gravity, rho)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('质量、阻力面积、倾角、重力与密度必须为有限数值')
        if mass <= 0.0:
            raise ValueError('质量必须为正')
        if drag_area <= 0.0:
            raise ValueError('阻力面积 Cd*A 必须为正（m^2）')
        if not 0.0 < max_tilt_deg < 90.0:
            raise ValueError('最大倾角必须在 (0, 90) 度之间')
        if gravity <= 0.0 or rho <= 0.0:
            raise ValueError('重力加速度与空气密度必须为正')
        self.mass = float(mass)
        self.drag_area = float(drag_area)
        self.max_tilt_deg = float(max_tilt_deg)
        self.gravity = float(gravity)
        self.rho = float(rho)

    # ------------------------------------------------------------------ 基本量

    @property
    def weight_n(self):
        return self.mass * self.gravity

    def drag_n(self, wind_speed_m_s):
        """风速 V 下的气动阻力 (N)."""
        if not math.isfinite(wind_speed_m_s) or wind_speed_m_s < 0.0:
            raise ValueError('风速必须是有限的非负数')
        return 0.5 * self.rho * self.drag_area * wind_speed_m_s ** 2

    def tilt_for_wind(self, wind_speed_m_s):
        """悬停在该风速下需要的倾角 (度)."""
        ratio = self.drag_n(wind_speed_m_s) / self.weight_n
        return math.degrees(math.atan(ratio))

    def wind_for_tilt(self, tilt_deg):
        """给定倾角能悬停的最大风速 (m/s)."""
        if not math.isfinite(tilt_deg) or not 0.0 <= tilt_deg < 90.0:
            raise ValueError('倾角必须在 [0, 90) 度之间')
        ratio = math.tan(math.radians(tilt_deg)) * self.weight_n / (
            0.5 * self.rho * self.drag_area)
        return math.sqrt(ratio)

    @property
    def max_wind_m_s(self):
        """最大倾角对应的抗风上限 (m/s)."""
        return self.wind_for_tilt(self.max_tilt_deg)

    @property
    def thrust_to_weight(self):
        """维持最大倾角悬停所需的最小推重比 T/W = 1 / cos(theta_max)."""
        return 1.0 / math.cos(math.radians(self.max_tilt_deg))

    @property
    def thrust_increase_percent(self):
        """相对竖直悬停需要多付出的推力 (%)."""
        return 100.0 * (self.thrust_to_weight - 1.0)

    def thrust_to_weight_at(self, wind_speed_m_s):
        """在给定风速下悬停所需的推重比."""
        return 1.0 / math.cos(math.radians(self.tilt_for_wind(wind_speed_m_s)))

    def gust_tilt_deg(self, wind_speed_m_s, gust_delta_m_s):
        """阵风增大 gust_delta_m_s 时，需要再倾斜多少度."""
        if not math.isfinite(gust_delta_m_s) or gust_delta_m_s < 0.0:
            raise ValueError('阵风增量必须是有限的非负数')
        base = self.tilt_for_wind(wind_speed_m_s)
        peak = self.tilt_for_wind(wind_speed_m_s + gust_delta_m_s)
        return peak - base

    def tilt_sweep(self, points=7):
        """0..theta_max 之间均匀取点的 (倾角, 风速) 列表."""
        if points < 2:
            raise ValueError('至少取 2 个点')
        step = self.max_tilt_deg / (points - 1)
        return [(index * step, self.wind_for_tilt(index * step)) for index in range(points)]

    def report(self):
        """汇总成可直接打印/写 JSON 的字典."""
        sweep = self.tilt_sweep()
        gust_wind = 3.0
        gust = self.gust_tilt_deg(self.max_wind_m_s, gust_wind)
        return {
            'parameters': {
                'mass_kg': round(self.mass, 4),
                'drag_area_m2': round(self.drag_area, 6),
                'max_tilt_deg': round(self.max_tilt_deg, 4),
                'air_density_kg_m3': round(self.rho, 6),
                'gravity_m_s2': self.gravity,
            },
            'weight_n': round(self.weight_n, 4),
            'max_wind_m_s': round(self.max_wind_m_s, 4),
            'max_wind_beaufort': beaufort_level(self.max_wind_m_s),
            'thrust_to_weight': round(self.thrust_to_weight, 4),
            'thrust_increase_percent': round(self.thrust_increase_percent, 2),
            'tilt_sweep': [{'tilt_deg': round(tilt, 3), 'wind_m_s': round(wind, 4)}
                           for tilt, wind in sweep],
            'gust_test': {
                'base_wind_m_s': round(self.max_wind_m_s, 4),
                'gust_delta_m_s': gust_wind,
                'extra_tilt_deg': round(gust, 3),
                'total_tilt_deg': round(self.max_tilt_deg + gust, 3),
            },
        }
