"""PD expert controller for antagonist muscle activation labels."""

from dataclasses import dataclass
from math import isfinite
from typing import Tuple

from .safety import ActivationLimiter


@dataclass(frozen=True)
class ControlState:
    target_angle: float
    angle: float
    angular_velocity: float
    flexor_force: float = 0.0
    extensor_force: float = 0.0

    def validate(self) -> None:
        if not all(isfinite(value) for value in vars(self).values()):
            raise ValueError("control state values must be finite")


class AntagonisticPDController:
    """Map a joint-space PD torque to bounded antagonist activations.

    Positive torque activates the flexor and negative torque activates the
    extensor. A small co-contraction term keeps both tendons engaged. The
    neural controller trained later imitates these expert actions using joint
    state and measured tendon-force feedback.
    """

    def __init__(
        self,
        kp: float = 12.0,
        kd: float = 0.5,
        torque_scale: float = 2.0,
        coactivation: float = 0.03,
        limiter: ActivationLimiter | None = None,
    ) -> None:
        if kp <= 0.0 or kd < 0.0 or torque_scale <= 0.0:
            raise ValueError("PD gains and torque scale must be positive")
        if not 0.0 <= coactivation <= 1.0:
            raise ValueError("coactivation must be in [0, 1]")
        self.kp = float(kp)
        self.kd = float(kd)
        self.torque_scale = float(torque_scale)
        self.coactivation = float(coactivation)
        self.limiter = limiter or ActivationLimiter()

    def command(self, state: ControlState) -> Tuple[float, float]:
        state.validate()
        torque = self.kp * (state.target_angle - state.angle)
        torque -= self.kd * state.angular_velocity
        normalized = torque / self.torque_scale
        flexor = self.coactivation + max(0.0, normalized)
        extensor = self.coactivation + max(0.0, -normalized)
        return self.limiter.apply((flexor, extensor))
