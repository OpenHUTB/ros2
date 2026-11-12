"""车辆、轮胎与 ESC 的参数定义。

所有量均为 SI 单位：m、kg、s、N、rad。
默认值取自一辆 1.5 t 的前驱紧凑型轿车（轴距 2.5 m、质心高 0.55 m）。
"""
from __future__ import annotations

from dataclasses import dataclass, field

G = 9.80665


@dataclass(frozen=True)
class VehicleSpec:
    """双轨（two-track）车辆参数。"""

    mass: float = 1500.0
    """整车质量 m，kg。"""

    yaw_inertia: float = 2500.0
    """绕垂直轴的转动惯量 Iz，kg·m²。"""

    cg_to_front: float = 1.15
    """质心到前轴距离 a，m。"""

    cg_to_rear: float = 1.35
    """质心到后轴距离 b，m。"""

    track_front: float = 1.55
    """前轮距，m。"""

    track_rear: float = 1.55
    """后轮距，m。"""

    cg_height: float = 0.55
    """质心高度 h，m，用于载荷转移。"""

    wheel_radius: float = 0.31
    """车轮滚动半径，m。"""

    drag_area: float = 0.70
    """0.5·ρ·Cd·A，单位 N·s²/m²，用于纵向空气阻力。"""

    roll_split_front: float = 0.55
    """前轴侧倾刚度占比，决定侧向载荷转移在两轴间的分配。"""

    drive_force_max: float = 3400.0
    """单轮最大驱动力，N（前轮驱动，左右各一半）。"""

    brake_force_max: float = 9000.0
    """单轮最大制动力，N（不含附着限制）。"""

    # ---- 派生量 ----
    @property
    def wheelbase(self) -> float:
        return self.cg_to_front + self.cg_to_rear

    @property
    def weight(self) -> float:
        return self.mass * G

    def wheel_positions(self) -> tuple[tuple[float, float, bool], ...]:
        """四个车轮在车体坐标下的位置 (x 向前, y 向左, 是否前轮)。"""
        return (
            (self.cg_to_front, self.track_front / 2.0, True),   # 左前
            (self.cg_to_front, -self.track_front / 2.0, True),  # 右前
            (-self.cg_to_rear, self.track_rear / 2.0, False),   # 左后
            (-self.cg_to_rear, -self.track_rear / 2.0, False),  # 右后
        )


@dataclass(frozen=True)
class TireSpec:
    """简化魔术公式（Magic Formula）纵向/侧向轮胎模型参数。"""

    stiffness: float = 9.5
    """侧偏刚度因子 B，无量纲。

    峰值侧偏角 arctan(tan(pi/2C)/B) 在 B=9.5、C=1.35 时是 13.7°。真实乘用车轮胎
    一般在 6°~8°，这里取得小、轮胎偏「宽容」，所以侧向力在更大的侧偏角上才饱和。
    结论（有没有 ESC 的差别）不受影响，但绝对数值别直接对标实车。
    """

    shape: float = 1.35
    """曲线形状因子 C。"""

    mu_nominal: float = 0.9
    """名义附着系数，实际按路面分段给定。"""

    abs_ratio: float = 0.90
    """理想 ABS 允许的纵向力占附着极限的比例。

    真实车辆全力制动时车轮滑移率由 ABS 控制在峰值附近，纵向力不会
    顶到 μ·Fz，否则侧向能力被完全挤占。这里按 0.9 处理，
    与 ESC 文献里常用的「理想 ABS 假设」一致。
    """

    @property
    def peak_slip_angle(self) -> float:
        """峰值侧向力对应的侧偏角，rad。"""
        import math
        return math.atan(math.tan(math.pi / (2.0 * self.shape)) / self.stiffness)

    def lateral(self, slip_angle: float, mu: float, normal_force: float) -> float:
        """侧向力，N。slip_angle 用轮心速度定义，正号表示轮胎侧滑向左。"""
        import math
        peak = mu * normal_force
        return -peak * math.sin(self.shape * math.atan(self.stiffness * slip_angle))


@dataclass(frozen=True)
class EscConfig:
    """ESC 参数。"""

    enabled: bool = True

    yaw_rate_deadband: float = 0.02
    """横摆角速度误差死区，rad/s（0.02 约 1.1 deg/s）。

    取 0 时误差一离开零就介入，理论上响应最快，但 50 Hz 下会在零附近反复穿越，
    表现为左右轮交替加压。取 0.02 后方向翻转次数从 6 次降到 4 次，侧偏角只差 0.02 deg。
    """

    yaw_gain: float = 6.0
    """横摆角速度误差到制动强度的比例增益，1/(rad/s)。"""

    yaw_integral_gain: float = 2.0
    """误差积分增益，1/(rad/s)/s。用来消掉对开路面制动时的稳态横摆偏差。"""

    yaw_rate_gain: float = 0.12
    """误差变化率的微分增益，s/(rad/s)。"""

    sideslip_soft: float = 0.10
    """侧偏角软阈值，rad（约 5.7 deg），超过后按比例追加修正力矩。"""

    sideslip_gain: float = 4.0
    """侧偏角超限折算成等效横摆误差的系数，1/s。"""

    speed_brake_gain: float = 0.30
    """侧偏角超限时四轮同时追加的制动比例，用来把车速降回附着能力以内。"""

    intensity_tau: float = 0.0
    """控制强度的一阶低通时间常数，s。默认 0（不滤波）。

    误差每 20 ms 更新一次，不带滤波时强度会在 0.02～1.00 之间跳变。试过 0.03 s
    （约 1.5 个控制周期）：介入时长几乎不变（4.10 s 对 4.14 s），跟踪误差反而从
    2.10 涨到 2.16 deg/s，方向翻转从 6 次增到 8 次，所以默认不开。真正压住抖振的是
    yaw_rate_deadband。
    """

    throttle_cut: float = 0.35
    """制动强度超过该值时请求发动机降扭（前驱车降低前轮负荷）。"""

    split_mu_feedforward: float = 1.15
    """左右附着不同时，按附着差前馈一部分修正力矩，减轻积分负担。"""

    integral_limit: float = 0.45
    """积分项上限，rad，防风up。"""

    release_gain: float = 1.0
    """对产生反向横摆力矩的一侧，最多可释放掉的制动比例。"""

    brake_add_threshold: float = 0.40
    """驾驶员制动超过该值时，ESC 只减压、不再加压。"""

    add_limit: float = 1.0
    """追加制动的上限（占该轮附着极限的比例）。"""

    reference_margin: float = 0.90
    """参考横摆角速度受附着限制时的安全系数。"""

    understeer_gradient: float = 0.0
    """不足转向梯度 K，s²/m²。0 表示按纯运动学参考（r = u·δ/L）。"""


@dataclass
class VehicleState:
    """平面三自由度车辆状态。"""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    u: float = 20.0
    v: float = 0.0
    r: float = 0.0

    def as_tuple(self) -> tuple[float, float, float, float, float, float]:
        return (self.x, self.y, self.yaw, self.u, self.v, self.r)

    @classmethod
    def from_tuple(cls, values: tuple[float, ...]) -> "VehicleState":
        return cls(*values)

    @property
    def speed(self) -> float:
        import math
        return math.hypot(self.u, self.v)

    @property
    def sideslip(self) -> float:
        """车体侧偏角 β = atan(v/u)，rad，左偏为正。"""
        import math
        # 甩尾到车头横着走时 u 会过零，用下限而不是直接返回 0，
        # 否则曲线上会出现 90° 掉到 0° 的假台阶
        u = max(abs(self.u), 0.3)
        return math.atan2(self.v, u)


@dataclass
class WheelState:
    """单个车轮在一帧内的受力，用于画图和诊断。"""

    index: int
    steer: float = 0.0
    slip_angle: float = 0.0
    normal_force: float = 0.0
    mu: float = TireSpec.mu_nominal
    lateral_force: float = 0.0
    longitudinal_force: float = 0.0
    brake_command: float = 0.0
    saturated: bool = False


@dataclass
class ControlInput:
    """一个控制周期内的输入。"""

    steer: float = 0.0
    """前轮转角，rad。"""

    driver_brake: float = 0.0
    """驾驶员制动踏板，0~1。"""

    wheel_brake: list[float] = field(default_factory=lambda: [0.0] * 4)
    """ESC 处理之后每个车轮的实际制动指令，0~1。"""

    throttle: float = 0.0
    """油门开度，0~1（用于巡航保速）。"""

    esc_active: bool = False
    esc_intensity: float = 0.0
