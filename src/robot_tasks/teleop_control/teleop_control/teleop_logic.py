"""Pure velocity-command handling for keyboard teleoperation."""

import math
from typing import Optional, Tuple


Velocity = Tuple[float, float]
ZERO_VELOCITY: Velocity = (0.0, 0.0)


def decode_key(data: bytes) -> Optional[str]:
    """Normalize one terminal byte; return ``None`` when stdin reaches EOF."""
    if not data:
        return None
    return data.decode("utf-8", errors="ignore").lower()


def _positive_finite(name: str, value: float) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f"{name} must be a finite value greater than zero")
    return number


class TeleopCommandState:
    """Track the last keyboard command and expire it after a safety timeout."""

    def __init__(
        self,
        linear_speed: float = 0.5,
        angular_speed: float = 0.5,
        command_timeout_sec: float = 0.5,
    ) -> None:
        self.linear_speed = _positive_finite("linear_speed", linear_speed)
        self.angular_speed = _positive_finite("angular_speed", angular_speed)
        self.command_timeout_sec = _positive_finite(
            "command_timeout_sec", command_timeout_sec
        )
        self._velocity = ZERO_VELOCITY
        self._last_input_at: Optional[float] = None

    @property
    def velocity(self) -> Velocity:
        return self._velocity

    def handle_key(self, key: str, now: float) -> Velocity:
        """Apply one key at monotonic time ``now`` and return the new velocity."""
        timestamp = float(now)
        if not math.isfinite(timestamp):
            raise ValueError("now must be a finite monotonic timestamp")

        normalized = key.lower()
        commands = {
            "w": (self.linear_speed, 0.0),
            "s": (-self.linear_speed, 0.0),
            "a": (0.0, self.angular_speed),
            "d": (0.0, -self.angular_speed),
        }
        self._velocity = commands.get(normalized, ZERO_VELOCITY)
        self._last_input_at = timestamp if normalized in commands else None
        return self._velocity

    def stop(self) -> Velocity:
        self._velocity = ZERO_VELOCITY
        self._last_input_at = None
        return self._velocity

    def expire(self, now: float) -> Optional[Velocity]:
        """Return zero once the active command has exceeded its timeout."""
        timestamp = float(now)
        if not math.isfinite(timestamp):
            raise ValueError("now must be a finite monotonic timestamp")
        if self._last_input_at is None or self._velocity == ZERO_VELOCITY:
            return None
        if timestamp >= self._last_input_at + self.command_timeout_sec:
            return self.stop()
        return None
