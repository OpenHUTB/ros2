"""旋翼桨尖马赫数与桨盘载荷核算（只依赖标准库）.

三个量都是课本公式，用来快速判断一套桨机组合是否"太激进"：

1. 桨尖速度与马赫数
       v_tip = pi * D * rpm / 60,        M_tip = v_tip / a
   其中 a = sqrt(gamma * R * T) 是声速（15 摄氏度时约 340.3 m/s）。
   桨尖马赫数接近 0.7 后压缩性与噪声会明显变差，0.6 以上就要注意。

2. 桨盘载荷
       A_disk = pi * (D/2)^2 * n,        DL = m * g / A_disk
   同样的起飞重量，桨盘载荷越大，悬停诱导速度与功率越大。

3. 悬停诱导速度（动量理论）
       v_i = sqrt(m * g / (2 * rho * A_disk))

这些是选型/噪声评估用的静态估算，不含叶素理论、桨叶扭转、叶尖损失、涡流与气动干扰，
不能代替桨叶气动仿真或台架试验。
"""
import math

GAMMA = 1.4           # 空气比热比
R_SPECIFIC = 287.05   # 干空气气体常数 J/(kg*K)
T0_KELVIN = 288.15    # 15 摄氏度
RHO0 = 1.225          # 海平面标准大气密度 kg/m^3
G0 = 9.80665          # 标准重力加速度 m/s^2

# 判定阈值：桨尖马赫数与桨盘载荷
MACH_MARGIN = 0.6     # 0.6 以上提示
MACH_LIMIT = 0.7      # 0.7 以上桨尖压缩性明显
DISK_LOADING_LOW = 100.0    # N/m^2
DISK_LOADING_HIGH = 250.0   # N/m^2


def speed_of_sound(temperature_c=15.0):
    """声速 a = sqrt(gamma * R * T)，温度用摄氏度传入."""
    if not math.isfinite(temperature_c) or not -60.0 <= temperature_c <= 60.0:
        raise ValueError('温度必须在 -60..60 摄氏度之间')
    return math.sqrt(GAMMA * R_SPECIFIC * (temperature_c + 273.15))


def tip_speed(rpm, diameter_m):
    """桨尖线速度 (m/s): v_tip = pi * D * rpm / 60."""
    if not math.isfinite(rpm) or rpm < 0.0:
        raise ValueError('转速必须是非负有限值')
    if not math.isfinite(diameter_m) or diameter_m <= 0.0:
        raise ValueError('桨径必须为正')
    return math.pi * diameter_m * rpm / 60.0


def tip_mach(rpm, diameter_m, temperature_c=15.0):
    """桨尖马赫数 M_tip = v_tip / a."""
    return tip_speed(rpm, diameter_m) / speed_of_sound(temperature_c)


def disk_area(diameter_m, rotor_count=4):
    """总桨盘面积 (m^2): pi * (D/2)^2 * n."""
    if not math.isfinite(diameter_m) or diameter_m <= 0.0:
        raise ValueError('桨径必须为正')
    if not isinstance(rotor_count, int) or rotor_count < 1:
        raise ValueError('旋翼数量必须是 >= 1 的整数')
    return math.pi * (diameter_m / 2.0) ** 2 * rotor_count


def disk_loading(mass, diameter_m, rotor_count=4, gravity=G0):
    """桨盘载荷 W/A (N/m^2)."""
    if not math.isfinite(mass) or mass <= 0.0:
        raise ValueError('质量必须为正')
    if gravity <= 0.0:
        raise ValueError('重力加速度必须为正')
    return mass * gravity / disk_area(diameter_m, rotor_count)


def induced_velocity(mass, diameter_m, rotor_count=4, rho=RHO0, gravity=G0):
    """悬停诱导速度 v_i = sqrt(m*g / (2*rho*A)) (m/s)."""
    if not math.isfinite(rho) or rho <= 0.0:
        raise ValueError('空气密度必须为正')
    loading = disk_loading(mass, diameter_m, rotor_count, gravity)
    return math.sqrt(loading / (2.0 * rho))


def mach_level(mach):
    """按桨尖马赫数给出提示等级: ok / caution / too_high."""
    if not math.isfinite(mach) or mach < 0.0:
        raise ValueError('马赫数必须是非负有限值')
    if mach >= MACH_LIMIT:
        return 'too_high'
    if mach >= MACH_MARGIN:
        return 'caution'
    return 'ok'


def disk_loading_level(loading_n_m2):
    """按桨盘载荷给出提示等级: low / medium / high."""
    if not math.isfinite(loading_n_m2) or loading_n_m2 < 0.0:
        raise ValueError('桨盘载荷必须是非负有限值')
    if loading_n_m2 < DISK_LOADING_LOW:
        return 'low'
    if loading_n_m2 <= DISK_LOADING_HIGH:
        return 'medium'
    return 'high'


class PropCheck:
    """给定整机与桨参数的桨尖马赫数、桨盘载荷与诱导速度核算."""

    def __init__(self, mass=2.0, diameter_m=0.254, rotor_count=4,
                 hover_rpm=6500.0, max_rpm=9000.0, temperature_c=15.0,
                 rho=RHO0, gravity=G0):
        for name, value in (('mass', mass), ('diameter_m', diameter_m),
                            ('hover_rpm', hover_rpm), ('max_rpm', max_rpm),
                            ('temperature_c', temperature_c), ('rho', rho),
                            ('gravity', gravity)):
            if not math.isfinite(value):
                raise ValueError('%s 必须是有限数值' % name)
        if mass <= 0.0:
            raise ValueError('质量必须为正')
        if diameter_m <= 0.0:
            raise ValueError('桨径必须为正')
        if hover_rpm < 0.0 or max_rpm < 0.0:
            raise ValueError('转速必须是非负数')
        if max_rpm and hover_rpm > max_rpm:
            raise ValueError('悬停转速不能高于最大转速')
        if rho <= 0.0 or gravity <= 0.0:
            raise ValueError('空气密度与重力加速度必须为正')
        self.mass = float(mass)
        self.diameter_m = float(diameter_m)
        self.rotor_count = int(rotor_count)
        self.hover_rpm = float(hover_rpm)
        self.max_rpm = float(max_rpm)
        self.temperature_c = float(temperature_c)
        self.rho = float(rho)
        self.gravity = float(gravity)
        # 构造时就把面积相关量算好，非法输入尽早报错
        self.area_m2 = disk_area(self.diameter_m, self.rotor_count)
        self.sound_speed_m_s = speed_of_sound(self.temperature_c)

    # ------------------------------------------------------------------ 指标

    @property
    def weight_n(self):
        return self.mass * self.gravity

    @property
    def hover_tip_speed_m_s(self):
        return tip_speed(self.hover_rpm, self.diameter_m)

    @property
    def hover_tip_mach(self):
        return self.hover_tip_speed_m_s / self.sound_speed_m_s

    @property
    def max_tip_mach(self):
        if self.max_rpm <= 0.0:
            return 0.0
        return tip_speed(self.max_rpm, self.diameter_m) / self.sound_speed_m_s

    @property
    def disk_loading_n_m2(self):
        return self.weight_n / self.area_m2

    @property
    def disk_loading_kg_m2(self):
        return self.mass / self.area_m2

    @property
    def induced_velocity_m_s(self):
        return math.sqrt(self.disk_loading_n_m2 / (2.0 * self.rho))

    @property
    def thrust_per_rotor_n(self):
        return self.weight_n / self.rotor_count

    def rpm_for_mach(self, target_mach):
        """反解达到给定桨尖马赫数所需的转速 (rpm)."""
        if not math.isfinite(target_mach) or not 0.0 < target_mach < 1.0:
            raise ValueError('目标马赫数必须在 (0, 1) 之间')
        return target_mach * self.sound_speed_m_s * 60.0 / (math.pi * self.diameter_m)

    def diameter_for_disk_loading(self, target_n_m2):
        """反解在给定桨盘载荷下需要的桨径 (m)."""
        if not math.isfinite(target_n_m2) or target_n_m2 <= 0.0:
            raise ValueError('目标桨盘载荷必须为正')
        area_total = self.weight_n / target_n_m2
        area_per_rotor = area_total / self.rotor_count
        return 2.0 * math.sqrt(area_per_rotor / math.pi)

    def report(self):
        """汇总成可直接打印/写 JSON 的字典."""
        return {
            'parameters': {
                'mass_kg': round(self.mass, 4),
                'diameter_m': round(self.diameter_m, 4),
                'rotor_count': self.rotor_count,
                'hover_rpm': round(self.hover_rpm, 1),
                'max_rpm': round(self.max_rpm, 1),
                'temperature_c': round(self.temperature_c, 2),
                'air_density_kg_m3': round(self.rho, 6),
            },
            'speed_of_sound_m_s': round(self.sound_speed_m_s, 3),
            'weight_n': round(self.weight_n, 4),
            'thrust_per_rotor_n': round(self.thrust_per_rotor_n, 4),
            'total_disk_area_m2': round(self.area_m2, 6),
            'tip_speed_hover_m_s': round(self.hover_tip_speed_m_s, 3),
            'tip_mach_hover': round(self.hover_tip_mach, 4),
            'tip_mach_max': round(self.max_tip_mach, 4),
            'tip_mach_level': mach_level(self.max_tip_mach),
            'disk_loading_n_m2': round(self.disk_loading_n_m2, 3),
            'disk_loading_kg_m2': round(self.disk_loading_kg_m2, 4),
            'disk_loading_level': disk_loading_level(self.disk_loading_n_m2),
            'induced_velocity_m_s': round(self.induced_velocity_m_s, 4),
            'limits': {
                'rpm_for_mach_0_6': round(self.rpm_for_mach(MACH_MARGIN), 1),
                'rpm_for_mach_0_7': round(self.rpm_for_mach(MACH_LIMIT), 1),
                'diameter_for_100_n_m2': round(self.diameter_for_disk_loading(
                    DISK_LOADING_LOW), 4),
            },
            'notes': [
                '桨尖马赫数按最大转速评估；0.6 以上进入注意区，0.7 以上压缩性与噪声明显',
                '桨盘载荷与诱导速度是悬停值；前飞时诱导速度下降、型阻功率上升',
                '静态估算，不含叶素理论、桨叶扭转、叶尖损失与气动干扰',
            ],
        }
