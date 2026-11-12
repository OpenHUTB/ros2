"""列车编组纵向动力学参数（全部 SI 单位）。

默认单车参数刻意与已合并的 `train_ato_stop` 单质点模型对齐：
4 节 × 75 t = 300 t，启动牵引力 4 × 50 kN = 200 kN，牵引功率 4 × 0.8 MW = 3.2 MW，
Davis 系数 4 × (375, 5, 1.5) = (1500, 20, 6)。这样"单质点 vs 多编组"的差别
只来自车钩，而不是来自换了一列车。
"""

from dataclasses import dataclass

G = 9.81


@dataclass(frozen=True)
class CarParams:
    """单节车（动车）的纵向参数。"""

    mass: float = 75_000.0          # AW2 载荷下的单节质量
    f_start: float = 50_000.0       # 启动牵引力上限
    power: float = 800_000.0        # 牵引功率
    davis_a: float = 375.0          # Davis 常数项 N
    davis_b: float = 5.0            # Davis 一次项 N·s/m
    davis_c: float = 1.5            # Davis 二次项 N·s²/m²
    length: float = 22.0            # 车辆定距按车长处理，简化
    gamma: float = 1.06             # 回转质量系数

    def traction(self, v):
        """可用牵引力：低速恒力矩、高速恒功率。"""
        if v < 1e-3:
            return self.f_start
        return min(self.f_start, self.power / v)

    def resistance(self, v):
        """Davis 基本阻力。"""
        return self.davis_a + self.davis_b * v + self.davis_c * v * v


# 与 train_ato_stop 完全等价的"合并成一节车"的参数，用于单质点对照
SINGLE_BODY = CarParams(
    mass=300_000.0,
    f_start=200_000.0,
    power=3_200_000.0,
    davis_a=1500.0,
    davis_b=20.0,
    davis_c=6.0,
)

DEFAULT_CAR = CarParams()


@dataclass(frozen=True)
class CouplerParams:
    """车钩：线性弹簧 + 阻尼 + 间隙。

    刚度取 25 MN/m（密接式车钩缓冲器量级），阻尼 0.3 MN·s/m 对应阻尼比约 0.15，
    间隙 2 mm 用来产生"松-紧"过渡时的冲击。
    """

    stiffness: float = 25e6
    damping: float = 3.0e5
    gap: float = 0.002


@dataclass(frozen=True)
class ActuatorParams:
    """牵引/制动执行机构：一阶滞后。"""

    tau_traction: float = 0.30
    tau_brake: float = 0.80


@dataclass(frozen=True)
class ControlParams:
    """ATO 控制参数，与 train_ato_stop 的 AtoConfig 同口径。"""

    a_plan: float = 0.8             # 规划减速度
    a_service: float = 1.0          # 常用制动能力
    a_emergency: float = 1.2        # 紧急制动能力
    k_v: float = 1.2                # 速度跟踪增益
    speed_margin: float = 2.0 / 3.6 # 限速裕量 2 km/h
    stop_window: float = 40.0       # 进入停车闭环的距离
    stop_speed: float = 0.8         # 进入停车闭环的速度
    kp: float = 1.2                 # 停车闭环距离增益
    kd: float = 1.6                 # 停车闭环速度阻尼
    preview: float = 2.0            # 前向预瞄
    lead_time: float = 0.80         # 执行机构滞后的前馈补偿时间


@dataclass(frozen=True)
class Route:
    """线路：定长、单一定速、分段常数坡度（上坡为正）。"""

    length: float = 2400.0
    speed_limit: float = 160.0 / 3.6
    grades: tuple = ()              # ((x0, x1, 坡度小数), ...)

    def grade(self, x):
        for x0, x1, grade in self.grades:
            if x0 <= x < x1:
                return grade
        return 0.0

    def limit_at(self, x):
        return self.speed_limit
