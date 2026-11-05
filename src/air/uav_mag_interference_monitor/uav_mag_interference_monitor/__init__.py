"""uav_mag_interference_monitor：磁力计干扰检测与航向一致性门控。

纯判据层在 mag_interference_math.py，本包对外只做再导出，
方便 `from uav_mag_interference_monitor import Config, ...`。
"""

from .mag_interference_math import (
    CircleFit,
    Config,
    FlightSample,
    HeadingMonitor,
    HeadingOutput,
    HeadingState,
    MagSample,
    PreflightResult,
    PreflightState,
    fit_circle,
    run_preflight,
    wrap_angle,
)

__all__ = [
    "CircleFit",
    "Config",
    "FlightSample",
    "HeadingMonitor",
    "HeadingOutput",
    "HeadingState",
    "MagSample",
    "PreflightResult",
    "PreflightState",
    "fit_circle",
    "run_preflight",
    "wrap_angle",
]
