"""UAV yaw shortest-path and angular-rate guard."""

from .core import (
    YawGuard,
    YawGuardConfig,
    YawGuardResult,
    shortest_angle,
    wrap_angle,
)

__all__ = [
    "YawGuard",
    "YawGuardConfig",
    "YawGuardResult",
    "shortest_angle",
    "wrap_angle",
]
