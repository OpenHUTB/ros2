"""ROS2 水面无人艇低速定点保持（欠驱动 + 横漂补偿）。

核心包只依赖标准库与 NumPy，不 import rclpy；ROS2 节点在 ``node.py`` 中单独提供。
"""
from .allocator import AllocationResult, allocate, yaw_authority
from .control import ControlConfig, StationKeepingController, WrenchCommand
from .geometry import body_to_world, clamp, unit, world_to_body, wrap_angle
from .guidance import GuidanceConfig, GuidanceOutput, StationKeepingGuidance
from .monitor import (
    DEFAULT_POLICY,
    DEGRADED_POLICY,
    FAILED_POLICY,
    DegradedPolicy,
    FlowEstimator,
    FlowEstimatorConfig,
    HealthConfig,
    HealthLevel,
    MonitorSnapshot,
    ThrusterHealth,
    policy_for,
)
from .pilot import PilotCommand, PilotConfig, StationKeepingPilot, thrust_magnitude
from .vehicle import (
    ActuatorState,
    VehicleParams,
    VehicleState,
    integrate,
    relative_velocity,
    thrust_to_wrench,
)

__all__ = [
    "AllocationResult",
    "ActuatorState",
    "ControlConfig",
    "DEFAULT_POLICY",
    "DEGRADED_POLICY",
    "DegradedPolicy",
    "FAILED_POLICY",
    "FlowEstimator",
    "FlowEstimatorConfig",
    "GuidanceConfig",
    "GuidanceOutput",
    "HealthConfig",
    "HealthLevel",
    "MonitorSnapshot",
    "PilotCommand",
    "PilotConfig",
    "StationKeepingController",
    "StationKeepingGuidance",
    "StationKeepingPilot",
    "ThrusterHealth",
    "VehicleParams",
    "VehicleState",
    "WrenchCommand",
    "allocate",
    "body_to_world",
    "clamp",
    "integrate",
    "policy_for",
    "relative_velocity",
    "thrust_magnitude",
    "thrust_to_wrench",
    "unit",
    "world_to_body",
    "wrap_angle",
    "yaw_authority",
]
