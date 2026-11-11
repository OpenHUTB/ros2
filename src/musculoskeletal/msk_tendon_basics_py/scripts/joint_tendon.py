"""一对拮抗肌驱动的单自由度关节，以及“刹车-松手”释放实验。

关节几何::

    L_flex = L_rest - r * theta
    L_ext  = L_rest + r * theta

关节动力学（肌肉-肌腱在每个积分步内瞬间达到串联力平衡）::

    I * theta_ddot = r * (F_flex - F_ext)
                     - b * theta_dot
                     - k_load * (theta - theta_load_rest)

报告中的松手实验从 ``theta = 0.05 rad`` 开始，负载弹簧的自然位置也是
``0.05 rad``；这样松手前屈肌力恰为 89.5 N、肌腱储能为 0.134 J。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .muscle_tendon import (
    Equilibrium,
    MuscleTendon,
    MuscleTendonParams,
    activation_step,
)


@dataclass(frozen=True)
class JointParams:
    """关节与负载参数。"""

    moment_arm: float = 0.03       # r [m]
    inertia: float = 0.05          # I [kg m^2]
    damping: float = 0.20          # b [N m s/rad]
    rest_mtu: float = 0.135        # 中立位 MTU 长度 [m]
    load_stiffness: float = 6.0    # 负载弹簧 [N m/rad]
    release_angle: float = 0.05    # 松手时刻的关节角 [rad]
    load_rest_angle: float = 0.05  # 负载弹簧自然角 [rad]

    @classmethod
    def from_muscle_params(cls, p: MuscleTendonParams) -> "JointParams":
        return cls(
            moment_arm=p.moment_arm,
            inertia=p.inertia,
            damping=p.damping,
            rest_mtu=p.rest_mtu,
            load_stiffness=p.load_stiffness,
        )


@dataclass
class ReleaseResult:
    """松手实验的时间序列与关键指标。"""

    time: np.ndarray
    theta: np.ndarray
    omega: np.ndarray
    flex_activation: np.ndarray
    ext_activation: np.ndarray
    flex_force: np.ndarray
    ext_force: np.ndarray
    flex_energy: np.ndarray
    flex_fiber_length_norm: np.ndarray
    phase: np.ndarray
    hold_time: float
    pre_force: float
    pre_energy: float
    min_energy: float
    release_fraction: float
    peak_angle: float
    peak_time: float
    tendon_stiffness: float = field(default=float("nan"))

    def __len__(self) -> int:
        return int(self.time.size)


class AntagonistJoint:
    """屈肌 / 伸肌拮抗关节。"""

    def __init__(self, muscle_params: MuscleTendonParams | None = None,
                 joint_params: JointParams | None = None,
                 tendon_stiffness: float | None = None) -> None:
        self.muscle_params = (
            muscle_params if muscle_params is not None else MuscleTendonParams()
        )
        self.joint_params = (
            joint_params
            if joint_params is not None
            else JointParams.from_muscle_params(self.muscle_params)
        )
        self.tendon_stiffness = (
            self.muscle_params.k_t
            if tendon_stiffness is None
            else float(tendon_stiffness)
        )
        self.flexor = MuscleTendon(self.muscle_params, self.tendon_stiffness)
        self.extensor = MuscleTendon(self.muscle_params, self.tendon_stiffness)

    # ------------------------------------------------------------------
    # 几何与静力学
    # ------------------------------------------------------------------
    def mtu_lengths(self, theta: float) -> tuple[float, float]:
        r = self.joint_params.moment_arm
        l_rest = self.joint_params.rest_mtu
        return l_rest - r * theta, l_rest + r * theta

    def equilibria(self, theta: float, flex_activation: float,
                   ext_activation: float) -> tuple[Equilibrium, Equilibrium]:
        l_flex, l_ext = self.mtu_lengths(theta)
        eq_flex = self.flexor.equilibrium(l_flex, flex_activation)
        eq_ext = self.extensor.equilibrium(l_ext, ext_activation)
        return eq_flex, eq_ext

    def forces(self, theta: float, flex_activation: float,
               ext_activation: float) -> tuple[float, float]:
        eq_flex, eq_ext = self.equilibria(theta, flex_activation, ext_activation)
        return eq_flex.f_t, eq_ext.f_t

    def torque(self, theta: float, omega: float,
               flex_activation: float, ext_activation: float) -> float:
        f_flex, f_ext = self.forces(theta, flex_activation, ext_activation)
        jp = self.joint_params
        return (
            jp.moment_arm * (f_flex - f_ext)
            - jp.damping * omega
            - jp.load_stiffness * (theta - jp.load_rest_angle)
        )

    def derivative(self, state: np.ndarray, flex_activation: float,
                   ext_activation: float) -> np.ndarray:
        theta, omega = float(state[0]), float(state[1])
        return np.array(
            [omega, self.torque(theta, omega, flex_activation, ext_activation)
             / self.joint_params.inertia],
            dtype=float,
        )

    # ------------------------------------------------------------------
    # 松手释放实验
    # ------------------------------------------------------------------
    def simulate_release(self, hold_time: float = 0.4,
                         duration: float = 1.2,
                         dt: float = 1e-3,
                         flex_command: float = 0.9,
                         ext_command: float = 0.05,
                         theta0: float | None = None,
                         initial_activation: float = 0.0) -> ReleaseResult:
        """先刹车保持，再松开关节并积分到指定时长。

        前 ``hold_time`` 秒关节角固定（刹车），激活度从 0 按一阶动力学
        上升到指令值；随后关节自由，每步用 RK4 积分，肌肉力在每个
        子步内用二分法求串联平衡。
        """

        if hold_time < 0.0 or duration <= 0.0 or dt <= 0.0:
            raise ValueError("要求 hold_time >= 0, duration > 0, dt > 0")

        theta0 = self.joint_params.release_angle if theta0 is None else float(theta0)
        n_hold = max(0, int(round(hold_time / dt)))
        dt_hold = hold_time / n_hold if n_hold else 0.0
        n_rel = max(1, int(round(duration / dt)))

        times: list[float] = []
        thetas: list[float] = []
        omegas: list[float] = []
        a_flexs: list[float] = []
        a_exts: list[float] = []
        f_flexs: list[float] = []
        f_exts: list[float] = []
        energies: list[float] = []
        fiber_lengths: list[float] = []
        phases: list[str] = []

        a_flex = float(initial_activation)
        a_ext = float(initial_activation)
        theta = theta0
        omega = 0.0

        # ---- 刹车阶段 -------------------------------------------------
        for i in range(n_hold + 1):
            t = i * dt_hold if i < n_hold else hold_time
            eq_flex, eq_ext = self.equilibria(theta, a_flex, a_ext)
            times.append(t)
            thetas.append(theta)
            omegas.append(0.0)
            a_flexs.append(a_flex)
            a_exts.append(a_ext)
            f_flexs.append(eq_flex.f_t)
            f_exts.append(eq_ext.f_t)
            energies.append(eq_flex.tendon_energy)
            fiber_lengths.append(eq_flex.fiber_length_norm)
            phases.append("hold")
            if i < n_hold:
                a_flex = activation_step(
                    a_flex, flex_command, dt_hold,
                    self.muscle_params.tau_act, self.muscle_params.tau_deact,
                )
                a_ext = activation_step(
                    a_ext, ext_command, dt_hold,
                    self.muscle_params.tau_act, self.muscle_params.tau_deact,
                )

        # ---- 自由释放阶段 ---------------------------------------------
        for i in range(1, n_rel + 1):
            y = np.array([theta, omega], dtype=float)
            k1 = self.derivative(y, a_flex, a_ext)
            k2 = self.derivative(y + 0.5 * dt * k1, a_flex, a_ext)
            k3 = self.derivative(y + 0.5 * dt * k2, a_flex, a_ext)
            k4 = self.derivative(y + dt * k3, a_flex, a_ext)
            y = y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
            theta, omega = float(y[0]), float(y[1])
            t = hold_time + i * dt

            eq_flex, eq_ext = self.equilibria(theta, a_flex, a_ext)
            times.append(t)
            thetas.append(theta)
            omegas.append(omega)
            a_flexs.append(a_flex)
            a_exts.append(a_ext)
            f_flexs.append(eq_flex.f_t)
            f_exts.append(eq_ext.f_t)
            energies.append(eq_flex.tendon_energy)
            fiber_lengths.append(eq_flex.fiber_length_norm)
            phases.append("release")

        time = np.asarray(times, dtype=float)
        theta_arr = np.asarray(thetas, dtype=float)
        omega_arr = np.asarray(omegas, dtype=float)
        f_flex_arr = np.asarray(f_flexs, dtype=float)
        energy_arr = np.asarray(energies, dtype=float)
        fiber_arr = np.asarray(fiber_lengths, dtype=float)

        idx_hold = n_hold
        pre_force = float(f_flex_arr[idx_hold])
        pre_energy = float(energy_arr[idx_hold])
        release_energy = energy_arr[idx_hold:]
        min_energy = float(np.min(release_energy)) if release_energy.size else pre_energy
        release_fraction = (
            (pre_energy - min_energy) / pre_energy if pre_energy > 0.0 else float("nan")
        )
        release_theta = theta_arr[idx_hold:]
        peak_idx = int(np.argmax(release_theta))
        peak_angle = float(release_theta[peak_idx])
        peak_time = float(time[idx_hold + peak_idx])

        return ReleaseResult(
            time=time,
            theta=theta_arr,
            omega=omega_arr,
            flex_activation=np.asarray(a_flexs, dtype=float),
            ext_activation=np.asarray(a_exts, dtype=float),
            flex_force=f_flex_arr,
            ext_force=np.asarray(f_exts, dtype=float),
            flex_energy=energy_arr,
            flex_fiber_length_norm=fiber_arr,
            phase=np.asarray(phases, dtype=object),
            hold_time=float(hold_time),
            pre_force=pre_force,
            pre_energy=pre_energy,
            min_energy=min_energy,
            release_fraction=float(release_fraction),
            peak_angle=peak_angle,
            peak_time=peak_time,
            tendon_stiffness=self.tendon_stiffness,
        )
