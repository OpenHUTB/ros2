"""Joint-space controller with six-muscle activation synergies."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Tuple

from .kinematics import ArmGeometry, inverse_kinematics
from .safety import ActivationLimiter


MUSCLE_NAMES = (
    'shoulder_flexor',
    'shoulder_extensor',
    'elbow_flexor',
    'elbow_extensor',
    'biarticular_flexor',
    'biarticular_extensor',
)


@dataclass(frozen=True)
class ArmControlState:
    target_x_m: float
    target_z_m: float
    shoulder_rad: float
    elbow_rad: float
    shoulder_velocity_rad_s: float
    elbow_velocity_rad_s: float

    def validate(self) -> None:
        if not all(isfinite(value) for value in vars(self).values()):
            raise ValueError('arm control state values must be finite')


class MuscleArmController:
    """Map a Cartesian target to six bounded muscle activations."""

    def __init__(
        self,
        geometry: ArmGeometry = ArmGeometry(),
        shoulder_kp: float = 32.0,
        shoulder_kd: float = 2.0,
        elbow_kp: float = 17.0,
        elbow_kd: float = 1.6,
        shoulder_torque_scale: float = 3.0,
        elbow_torque_scale: float = 2.0,
        biarticular_torque_scale: float = 4.0,
        coactivation: float = 0.025,
    ) -> None:
        if min(
            shoulder_kp,
            elbow_kp,
            shoulder_torque_scale,
            elbow_torque_scale,
            biarticular_torque_scale,
        ) <= 0.0:
            raise ValueError('gains and torque scales must be positive')
        if shoulder_kd < 0.0 or elbow_kd < 0.0:
            raise ValueError('damping gains must be non-negative')
        if not 0.0 <= coactivation <= 1.0:
            raise ValueError('coactivation must be in [0, 1]')
        self.geometry = geometry
        self.shoulder_kp = float(shoulder_kp)
        self.shoulder_kd = float(shoulder_kd)
        self.elbow_kp = float(elbow_kp)
        self.elbow_kd = float(elbow_kd)
        self.shoulder_torque_scale = float(shoulder_torque_scale)
        self.elbow_torque_scale = float(elbow_torque_scale)
        self.biarticular_torque_scale = float(biarticular_torque_scale)
        self.coactivation = float(coactivation)
        self.limiter = ActivationLimiter(len(MUSCLE_NAMES))

    def command(self, state: ArmControlState) -> Tuple[float, ...]:
        state.validate()
        target_shoulder, target_elbow = inverse_kinematics(
            state.target_x_m,
            state.target_z_m,
            self.geometry,
        )
        shoulder_torque = (
            self.shoulder_kp * (target_shoulder - state.shoulder_rad)
            - self.shoulder_kd * state.shoulder_velocity_rad_s
        )
        elbow_torque = (
            self.elbow_kp * (target_elbow - state.elbow_rad)
            - self.elbow_kd * state.elbow_velocity_rad_s
        )
        shoulder_norm = shoulder_torque / self.shoulder_torque_scale
        elbow_norm = elbow_torque / self.elbow_torque_scale
        biarticular_norm = (
            shoulder_torque + elbow_torque
        ) / (2.0 * self.biarticular_torque_scale)

        command = (
            self.coactivation + max(0.0, shoulder_norm),
            self.coactivation + max(0.0, -shoulder_norm),
            self.coactivation + max(0.0, elbow_norm),
            self.coactivation + max(0.0, -elbow_norm),
            self.coactivation + max(0.0, biarticular_norm),
            self.coactivation + max(0.0, -biarticular_norm),
        )
        return self.limiter.apply(command)
