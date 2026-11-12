"""多旋翼重心偏移配平与饱和包线核算（只依赖标准库）."""

from .core import (
    G,
    UavCgTrim,
    margin_pct,
    per_motor_thrusts,
    saturation_envelope,
    trim_tilt_deg,
)

__all__ = ['UavCgTrim', 'per_motor_thrusts', 'trim_tilt_deg', 'saturation_envelope',
           'margin_pct', 'G']
