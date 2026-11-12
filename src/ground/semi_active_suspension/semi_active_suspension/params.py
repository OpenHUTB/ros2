"""整车垂向模型的参数定义。

车身只保留三个刚体自由度（垂向 z、俯仰 theta、侧倾 phi），四个车轮各有独立
的垂向自由度，合起来是 7 自由度。参数取一辆 1.5 t 的紧凑型轿车，量级和
``vehicle_dynamics_test`` 里的整车参数对得上。
"""
from __future__ import annotations

from dataclasses import dataclass, field

GRAVITY = 9.81

# 四个车轮的编号：左前、右前、左后、右后
FL, FR, RL, RR = 0, 1, 2, 3
CORNERS = ("FL", "FR", "RL", "RR")


@dataclass(frozen=True)
class VehicleSpec:
    """整车与悬架的物理参数。"""

    sprung_mass: float = 1450.0      # 簧上质量，kg
    unsprung_mass: float = 42.0      # 单个非簧载质量（轮胎+轮毂），kg
    inertia_pitch: float = 2450.0    # 俯仰转动惯量，kg*m^2
    inertia_roll: float = 620.0      # 侧倾转动惯量，kg*m^2

    cg_to_front: float = 1.15        # 质心到前轴，m
    cg_to_rear: float = 1.35         # 质心到后轴，m
    track: float = 1.55              # 轮距，m
    cg_height: float = 0.55          # 质心高度，m
    roll_center_height: float = 0.08  # 侧倾中心高度，m

    spring_front: float = 32000.0    # 前螺旋弹簧刚度，N/m
    spring_rear: float = 29000.0     # 后螺旋弹簧刚度，N/m
    tire_stiffness: float = 210000.0  # 轮胎垂向刚度，N/m
    tire_damping: float = 180.0      # 轮胎垂向阻尼，N*s/m

    damper_passive_front: float = 2400.0   # 被动减振器阻尼，N*s/m
    damper_passive_rear: float = 2100.0
    damper_min: float = 500.0              # 半主动可调下限，N*s/m
    damper_max: float = 6500.0             # 半主动可调上限，N*s/m

    @property
    def wheelbase(self) -> float:
        return self.cg_to_front + self.cg_to_rear

    @property
    def sprung_weight(self) -> float:
        return self.sprung_mass * GRAVITY

    def axle_position(self, corner: int) -> float:
        """车轮纵向位置，前轴为正。"""
        return self.cg_to_front if corner in (FL, FR) else -self.cg_to_rear

    def lateral_position(self, corner: int) -> float:
        """车轮横向位置，左侧为正。"""
        return self.track / 2.0 if corner in (FL, RL) else -self.track / 2.0

    def spring_rate(self, corner: int) -> float:
        return self.spring_front if corner in (FL, FR) else self.spring_rear

    def passive_damping(self, corner: int) -> float:
        return (self.damper_passive_front if corner in (FL, FR)
                else self.damper_passive_rear)

    def static_load(self, corner: int) -> float:
        """静态轮荷，N。按前后轴分配，左右均分。"""
        front_share = self.cg_to_rear / self.wheelbase
        share = front_share if corner in (FL, FR) else 1.0 - front_share
        return self.sprung_weight * share / 2.0 + self.unsprung_mass * GRAVITY


@dataclass(frozen=True)
class DamperConfig:
    """半主动阻尼的控制参数。"""

    skyhook_gain: float = 4200.0    # 天棚阻尼系数，N*s/m
    groundhook_gain: float = 1200.0  # 地棚分量，抑制轮胎动载
    valve_tau: float = 0.02          # 阻尼阀响应时间常数，s

    def clamp_low(self, spec: VehicleSpec) -> float:
        return spec.damper_min

    def clamp_high(self, spec: VehicleSpec) -> float:
        return spec.damper_max


@dataclass
class VehicleState:
    """7 自由度状态量。"""

    z_s: float = 0.0          # 车身垂向位移，m（向上为正）
    v_s: float = 0.0          # 车身垂向速度，m/s
    pitch: float = 0.0        # 俯仰角，rad（正 = 车头下沉）
    pitch_rate: float = 0.0
    roll: float = 0.0         # 侧倾角，rad（正 = 右侧下沉）
    roll_rate: float = 0.0
    z_u: tuple = field(default_factory=lambda: (0.0, 0.0, 0.0, 0.0))
    v_u: tuple = field(default_factory=lambda: (0.0, 0.0, 0.0, 0.0))

    def copy(self) -> "VehicleState":
        return VehicleState(self.z_s, self.v_s, self.pitch, self.pitch_rate,
                            self.roll, self.roll_rate, tuple(self.z_u),
                            tuple(self.v_u))


@dataclass(frozen=True)
class Disturbance:
    """外部激励：路面输入 + 车身受到的纵向/侧向加速度。"""

    road: tuple = (0.0, 0.0, 0.0, 0.0)   # 四个轮心的路面高度，m
    road_rate: tuple = (0.0, 0.0, 0.0, 0.0)
    accel_x: float = 0.0                 # 纵向加速度，m/s^2（制动为负）
    accel_y: float = 0.0                 # 侧向加速度，m/s^2（左转为正）
