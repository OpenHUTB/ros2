"""横向控制器：纯跟踪、Stanley 与运动学 LQRI（带横向偏差积分）。

统一约定
--------
* 车辆参考点为后轴中心。
* 横向偏差 e > 0 表示车辆在参考路径左侧。
* 航向误差 psi_e = psi − psi_ref（车头相对路径切向，左转为正）。
* 前轮转角 delta > 0 表示左转。

LQR 的线性化对象是**实际被控的运动学模型**（后轴自行车模型），而不是另一套
动力学模型——这一点很关键：用动力学模型设计增益、却在运动学模型上闭环，
等于拿错被控对象，闭环会直接发散。

    ė        = v · psi_e
    psi_e_dot = (v / L) · delta − v · kappa
    ė_I      = e

写成状态 x = [e, psi_e, e_I]、控制 u = delta：

    A = [[0, v, 0], [0, 0, v/L], [1, 0, 0]]
    B = [[0], [v/L], [0]]

曲率前馈 delta_ff = atan(L·kappa) 负责稳态；积分项负责消掉剩余的常值偏差。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .lqr_solver import solve_care_hamiltonian
from .path import Path, Projection, wrap_angle
from .vehicle import VehicleParams


def _pose_xy(pose):
    return float(pose[0]), float(pose[1]), float(pose[2]), float(pose[3])


def kinematic_error_model(v: float, params: VehicleParams):
    """后轴自行车模型在路径坐标下的误差线性化（3 状态）。"""
    v = max(float(v), 1.0)  # 低速时避免 B 矩阵退化
    a = np.array(
        [[0.0, v, 0.0], [0.0, 0.0, v / params.wheelbase], [1.0, 0.0, 0.0]]
    )
    b = np.array([[0.0], [v / params.wheelbase], [0.0]])
    return a, b


class BaseController:
    name = "base"
    label = "Base"

    def __init__(self, params: VehicleParams):
        self.params = params

    def reset(self):
        pass

    def compute(self, pose, projection: Projection, path: Path, dt: float) -> float:
        raise NotImplementedError


class PurePursuit(BaseController):
    """速度自适应前视距离的几何纯跟踪。

    Ld = Ld0 + k * v
    delta = atan2(2 L sin(alpha), Ld)
    """

    name = "pure_pursuit"
    label = "Pure Pursuit"

    def __init__(self, params: VehicleParams, ld0: float = 2.4, k_ld: float = 0.45):
        super().__init__(params)
        self.ld0 = float(ld0)
        self.k_ld = float(k_ld)

    def lookahead_distance(self, v: float) -> float:
        return self.ld0 + self.k_ld * max(float(v), 0.0)

    def compute(self, pose, projection: Projection, path: Path, dt: float) -> float:
        x, y, psi, v = _pose_xy(pose)
        ld = self.lookahead_distance(v)
        target_s = min(projection.s + ld, path.length)
        target = path.pose_at(target_s)

        dx_world = target.x - x
        dy_world = target.y - y
        cos_psi = np.cos(psi)
        sin_psi = np.sin(psi)
        dx = cos_psi * dx_world + sin_psi * dy_world
        dy = -sin_psi * dx_world + cos_psi * dy_world
        chord = float(np.hypot(dx, dy))
        if chord < 1e-6:
            return 0.0
        alpha = float(np.arctan2(dy, dx))
        # 分母使用实际弦长，等价于在离散步长下严格满足几何关系。
        return float(np.arctan2(2.0 * self.params.wheelbase * np.sin(alpha), chord))


class Stanley(BaseController):
    """Stanley 前轮转角控制。

    delta = psi_e + atan2(k * e_right, v) + atan(L * kappa)
    e_right = -e_left，因此左偏时横向前馈项为负，车辆向右修正。
    默认控制点取后轴（与车辆模型参考点一致）；切换到前轴可复现经典写法。
    """

    name = "stanley"
    label = "Stanley"

    def __init__(self, params: VehicleParams, k: float = 2.2,
                 use_rear_axle: bool = True, curvature_feedforward: bool = True,
                 v_soft: float = 0.6):
        super().__init__(params)
        self.k = float(k)
        self.use_rear_axle = bool(use_rear_axle)
        self.curvature_feedforward = bool(curvature_feedforward)
        self.v_soft = float(v_soft)

    def compute(self, pose, projection: Projection, path: Path, dt: float) -> float:
        x, y, psi, v = _pose_xy(pose)
        if self.use_rear_axle:
            control_point = np.array([x, y])
            ctrl_proj = projection
        else:
            control_point = np.array(
                [x + self.params.wheelbase * np.cos(psi),
                 y + self.params.wheelbase * np.sin(psi)]
            )
            ctrl_proj = path.project(float(control_point[0]), float(control_point[1]),
                                     s_hint=projection.s)

        e_left = ctrl_proj.e
        psi_e = float(wrap_angle(ctrl_proj.psi - psi))
        speed = max(float(v), self.v_soft)

        ff = 0.0
        if self.curvature_feedforward:
            ff = float(np.arctan(self.params.wheelbase * ctrl_proj.kappa))

        # 经典公式中的 e 采用“右偏为正”；这里把左偏为正的 e_left 取负。
        delta = psi_e + float(np.arctan2(self.k * (-e_left), speed)) + ff
        return float(delta)


class LQRController(BaseController):
    """运动学 LQRI：状态反馈 + 曲率前馈 + 横向偏差积分。

    三处工程细节：
    1. 误差在进入反馈前先限幅（e_clip），避免 1 m 以上的初始偏差让增益
       直接把转角顶到 ±35° 饱和、把车“拐”出路面；
    2. 积分只在中等误差区间内累积，并做幅值限幅（抗饱和）；
    3. 增益随速度重算——A、B 都含 v，定速增益换速会变差。
    """

    name = "lqr"
    label = "LQR"

    def __init__(self, params: VehicleParams, q_diag=None, r_weight: float = 1.0,
                 e_clip: float = 0.5, integral_limit: float = 0.3,
                 integration_deadband: float = 0.01,
                 integration_gate: float = 0.75,
                 design_interval: float = 0.2, v_change: float = 0.35):
        super().__init__(params)
        if q_diag is None:
            q_diag = (1.0, 1.0, 0.05)
        self.Q = np.diag(np.asarray(q_diag, dtype=float))
        self.R = np.array([[float(r_weight)]])
        self.e_clip = float(e_clip)
        self.integral_limit = float(integral_limit)
        self.integration_deadband = float(integration_deadband)
        self.integration_gate = float(integration_gate)
        self.design_interval = float(design_interval)
        self.v_change = float(v_change)

        self.integral = 0.0
        self.K = np.zeros((1, 3), dtype=float)
        self.P = np.eye(3)
        self._design_v = None
        self._since_design = 1e9
        self.riccati_residual = np.inf
        self.closed_loop_eigs = np.zeros(0)

    def reset(self):
        self.integral = 0.0
        self.K = np.zeros((1, 3), dtype=float)
        self.P = np.eye(3)
        self._design_v = None
        self._since_design = 1e9
        self.riccati_residual = np.inf
        self.closed_loop_eigs = np.zeros(0)

    def update_gain(self, v: float, force: bool = False):
        if not force and self._design_v is not None:
            if (abs(v - self._design_v) < self.v_change
                    and self._since_design < self.design_interval):
                return
        a, b = kinematic_error_model(v, self.params)
        solution = solve_care_hamiltonian(a, b, self.Q, self.R)
        self.K = solution["K"]
        self.P = solution["P"]
        self.riccati_residual = solution["residual_norm"]
        self.closed_loop_eigs = solution["closed_loop_eigs"]
        self._design_v = float(v)
        self._since_design = 0.0

    def compute(self, pose, projection: Projection, path: Path, dt: float) -> float:
        x, y, psi, v = _pose_xy(pose)
        self._since_design += dt
        self.update_gain(v)

        psi_e = float(wrap_angle(psi - projection.psi))
        e = float(projection.e)
        e_fb = float(np.clip(e, -self.e_clip, self.e_clip))
        state = np.array([e_fb, psi_e, self.integral], dtype=float)

        delta_fb = float(-(self.K @ state).item())
        delta_ff = float(np.arctan(self.params.wheelbase * projection.kappa))
        delta = float(np.clip(delta_ff + delta_fb,
                              -self.params.max_steer, self.params.max_steer))

        # 抗饱和积分：只在中等误差、且控制未深度饱和时累积。
        saturated = abs(delta) >= 0.985 * self.params.max_steer
        if (self.integration_deadband < abs(e) < self.integration_gate
                and not saturated):
            self.integral = float(np.clip(
                self.integral + e * dt, -self.integral_limit, self.integral_limit
            ))
        else:
            # 大偏差阶段让积分缓慢泄放，避免转回中线后还被旧积分拖着。
            self.integral *= max(0.0, 1.0 - dt / 0.5)
        return delta


def make_controllers(params: VehicleParams):
    """默认控制器组合；每次 bench/single 都创建新实例，避免积分状态串场。"""
    return [
        PurePursuit(params),
        Stanley(params, use_rear_axle=True),
        LQRController(params),
    ]
