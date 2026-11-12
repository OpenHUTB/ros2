"""Right-wall PID follower core (pure Python, no ROS dependency).

Kept free of ``rclpy`` so that ``tests/test_core.py`` can run with the system
Python without a ROS 2 installation.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class CmdVel:
    linear: float
    angular: float
    state: str


class PIDController:
    """Bumpless PID with integral clamping."""

    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0,
                 integral_max: float = 5.0) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_max = integral_max
        self._integral = 0.0
        self._last_error = None

    def reset(self) -> None:
        self._integral = 0.0
        self._last_error = None

    def update(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0
        self._integral = max(
            -self.integral_max,
            min(self.integral_max, self._integral + error * dt),
        )
        derivative = 0.0 if self._last_error is None else (error - self._last_error) / dt
        self._last_error = error
        return self.kp * error + self.ki * self._integral + self.kd * derivative


class WallFollower:
    """Reactive right-side wall follower for a differential-drive robot.

    States
    ------
    FOLLOW
        Keep ``target_distance`` to the right wall; move forward slowly.
    AVOID
        Forward distance below ``forward_safety`` -> rotate in place until
        the corridor ahead is clear again.
    """

    def __init__(self,
                 target_distance: float = 0.5,
                 forward_safety: float = 0.4,
                 max_linear: float = 0.3,
                 max_angular: float = 1.0,
                 kp: float = 1.2,
                 ki: float = 0.0,
                 kd: float = 0.15) -> None:
        self.target_distance = target_distance
        self.forward_safety = forward_safety
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.pid = PIDController(kp=kp, ki=ki, kd=kd)
        self.state = "FOLLOW"

    @staticmethod
    def _clip(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def compute(self, side_distance, forward_distance, dt: float) -> CmdVel:
        """Return :class:`CmdVel` for the latest distances.

        Parameters
        ----------
        side_distance:
            Measured distance to the right wall (metres). ``None`` or <=0 is
            treated as "no valid reading" and the controller holds the setpoint.
        forward_distance:
            Measured distance to the nearest obstacle in front (metres).
        dt:
            Time since the last call (seconds).
        """
        if forward_distance is None or forward_distance < 0:
            forward_distance = float("inf")

        # Forward obstacle: stop and rotate in place.
        if forward_distance < self.forward_safety:
            self.state = "AVOID"
            self.pid.reset()
            return CmdVel(0.0, self._clip(1.0, self.max_angular), self.state)

        # Continue rotating until the corridor is comfortable.
        if self.state == "AVOID" and forward_distance < self.forward_safety * 1.5:
            return CmdVel(0.0, self._clip(0.8, self.max_angular), self.state)

        self.state = "FOLLOW"
        if side_distance is None or side_distance <= 0:
            side_distance = self.target_distance

        error = self.target_distance - side_distance
        angular = self.pid.update(error, dt)
        # Slow down when the turn rate is large.
        linear = self.max_linear * max(
            0.2, 1.0 - abs(angular) / self.max_angular
        )
        return CmdVel(
            self._clip(linear, self.max_linear),
            self._clip(angular, self.max_angular),
            self.state,
        )
