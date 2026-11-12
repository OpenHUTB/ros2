"""ROS-independent wading-depth and buoyancy-margin calculation logic."""

from __future__ import annotations

from dataclasses import dataclass
import math


MODES = ("SAFE", "CAUTION", "FLOODED", "FLOATING")


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


@dataclass(frozen=True)
class WadingConfig:
    """Vehicle geometry and water-contact parameters.

    The vehicle is modelled as a rectangular box of ``body_length_m`` x
    ``body_width_m``. Only ``displacement_volume_factor`` of that box actually
    displaces water (tyres, subframe, sill volume), so a car does not float as
    soon as the water reaches its underbody.
    """

    mass_kg: float = 1500.0
    gravity: float = 9.81
    water_density: float = 1000.0
    body_length_m: float = 4.50
    body_width_m: float = 1.80
    body_height_m: float = 1.50
    intake_height_m: float = 0.65
    exhaust_height_m: float = 0.35
    door_sill_height_m: float = 0.40
    wheel_radius_m: float = 0.32
    safety_margin_ratio: float = 0.80
    displacement_volume_factor: float = 0.35
    drag_coefficient: float = 0.70
    wave_factor: float = 1.25
    crr_wet: float = 0.05
    friction_coef_dry: float = 0.60
    friction_coef_wet: float = 0.35
    float_ratio_limit: float = 0.90
    depth_timeout_s: float = 1.0
    speed_timeout_s: float = 1.0
    control_rate_hz: float = 20.0

    def __post_init__(self) -> None:
        for name in (
            "mass_kg",
            "gravity",
            "water_density",
            "body_length_m",
            "body_width_m",
            "body_height_m",
            "intake_height_m",
            "exhaust_height_m",
            "door_sill_height_m",
            "wheel_radius_m",
            "safety_margin_ratio",
            "displacement_volume_factor",
            "drag_coefficient",
            "wave_factor",
            "crr_wet",
            "friction_coef_dry",
            "friction_coef_wet",
            "float_ratio_limit",
            "depth_timeout_s",
            "speed_timeout_s",
            "control_rate_hz",
        ):
            _finite(getattr(self, name), name)
        for name in (
            "mass_kg",
            "gravity",
            "water_density",
            "body_length_m",
            "body_width_m",
            "body_height_m",
            "wheel_radius_m",
            "control_rate_hz",
        ):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be greater than zero")
        for name in (
            "intake_height_m",
            "exhaust_height_m",
            "door_sill_height_m",
            "displacement_volume_factor",
            "drag_coefficient",
            "wave_factor",
            "crr_wet",
        ):
            if getattr(self, name) < 0.0:
                raise ValueError(f"{name} must be non-negative")
        if not 0.0 < self.safety_margin_ratio <= 1.0:
            raise ValueError("safety_margin_ratio must be in (0, 1]")
        if not 0.0 < self.friction_coef_dry <= 1.5:
            raise ValueError("friction_coef_dry must be in (0, 1.5]")
        if not 0.0 < self.friction_coef_wet <= 1.5:
            raise ValueError("friction_coef_wet must be in (0, 1.5]")
        if not 0.0 < self.float_ratio_limit <= 1.0:
            raise ValueError("float_ratio_limit must be in (0, 1]")
        if self.depth_timeout_s <= 0.0 or self.speed_timeout_s <= 0.0:
            raise ValueError("timeouts must be greater than zero")


@dataclass(frozen=True)
class WadingResult:
    depth_m: float
    speed_mps: float
    mode: str
    depth_limit_m: float
    safe_depth_m: float
    float_depth_m: float
    buoyancy_n: float
    buoyancy_ratio: float
    traction_limit_n: float
    roll_resistance_n: float
    drag_n: float
    v_max_mps: float
    traction_margin_n: float
    brake_decay_ratio: float


def _depth_limit(config: WadingConfig) -> float:
    """Lowest opening (intake, exhaust, door sill) governs the depth limit."""
    return min(
        config.intake_height_m,
        config.exhaust_height_m,
        config.door_sill_height_m,
    )


def _dry_mass(config: WadingConfig) -> float:
    return config.mass_kg * config.gravity


def buoyancy_force(depth_m: float, config: WadingConfig) -> float:
    """Archimedes force on the sealed fraction of the body up to ``depth_m``."""
    if depth_m <= 0.0:
        return 0.0
    submerged_height = min(depth_m, config.body_height_m)
    volume = (
        config.body_length_m
        * config.body_width_m
        * submerged_height
        * config.displacement_volume_factor
    )
    return config.water_density * config.gravity * volume


def float_depth(config: WadingConfig) -> float:
    """Water depth at which buoyancy reaches ``float_ratio_limit`` of weight."""
    target_volume = (
        config.float_ratio_limit * config.mass_kg / config.water_density
    )
    area = (
        config.body_length_m
        * config.body_width_m
        * config.displacement_volume_factor
    )
    return target_volume / area


def evaluate_depth(depth_m: float, speed_mps: float, config: WadingConfig) -> WadingResult:
    """Evaluate one (depth, speed) sample against the wading limits."""
    _finite(depth_m, "depth_m")
    _finite(speed_mps, "speed_mps")
    if depth_m < 0.0:
        raise ValueError("depth_m must be non-negative")
    if speed_mps < 0.0:
        raise ValueError("speed_mps must be non-negative")

    depth_limit = _depth_limit(config)
    safe_depth = config.safety_margin_ratio * depth_limit
    float_d = float_depth(config)
    weight = _dry_mass(config)

    buoyancy = buoyancy_force(depth_m, config)
    buoyancy_ratio = buoyancy / weight if weight else 0.0
    effective_weight = max(0.0, weight - buoyancy)
    traction_limit = config.friction_coef_wet * effective_weight
    roll_resistance = config.crr_wet * effective_weight

    # Wetted frontal area of the body, floored so that a dry road does not
    # produce an infinite speed limit.
    wetted_area = config.body_width_m * max(depth_m, 0.05)
    drag_factor = (
        0.5
        * config.water_density
        * config.drag_coefficient
        * config.wave_factor
        * wetted_area
    )
    usable_force = traction_limit - roll_resistance
    if usable_force <= 0.0 or drag_factor <= 0.0:
        v_max = 0.0
    else:
        v_max = math.sqrt(usable_force / drag_factor)

    drag = drag_factor * speed_mps ** 2

    if buoyancy_ratio >= config.float_ratio_limit:
        mode = "FLOATING"
        v_max = 0.0
    elif depth_m > depth_limit:
        mode = "FLOODED"
    elif depth_m > safe_depth:
        mode = "CAUTION"
    else:
        mode = "SAFE"

    margin = usable_force - drag

    return WadingResult(
        depth_m=depth_m,
        speed_mps=speed_mps,
        mode=mode,
        depth_limit_m=depth_limit,
        safe_depth_m=safe_depth,
        float_depth_m=float_d,
        buoyancy_n=buoyancy,
        buoyancy_ratio=buoyancy_ratio,
        traction_limit_n=traction_limit,
        roll_resistance_n=roll_resistance,
        drag_n=drag,
        v_max_mps=v_max,
        traction_margin_n=margin,
        brake_decay_ratio=config.friction_coef_wet / config.friction_coef_dry,
    )


class WadingChecker:
    """Stateful checker with freshness windows for the two input signals."""

    def __init__(self, config: WadingConfig | None = None) -> None:
        self.config = config or WadingConfig()
        self._depth = 0.0
        self._speed = 0.0
        self._depth_time: float | None = None
        self._speed_time: float | None = None

    def update_depth(self, depth_m: float, now_s: float) -> None:
        _finite(depth_m, "depth_m")
        _finite(now_s, "now_s")
        if depth_m < 0.0:
            raise ValueError("depth_m must be non-negative")
        self._depth = depth_m
        self._depth_time = now_s

    def update_speed(self, speed_mps: float, now_s: float) -> None:
        _finite(speed_mps, "speed_mps")
        _finite(now_s, "now_s")
        if speed_mps < 0.0:
            raise ValueError("speed_mps must be non-negative")
        self._speed = speed_mps
        self._speed_time = now_s

    def reset(self) -> None:
        self._depth_time = None
        self._speed_time = None

    def step(self, now_s: float) -> WadingResult | None:
        """Return a result once both inputs are fresh, otherwise ``None``."""
        _finite(now_s, "now_s")
        if self._depth_time is None or self._speed_time is None:
            return None
        if now_s - self._depth_time > self.config.depth_timeout_s:
            return None
        if now_s - self._speed_time > self.config.speed_timeout_s:
            return None
        return evaluate_depth(self._depth, self._speed, self.config)

    def sweep(self, max_depth_m: float, step_m: float = 0.02) -> list[WadingResult]:
        """Duty-cycle sweep used by the report: v_max against water depth."""
        if max_depth_m <= 0.0:
            raise ValueError("max_depth_m must be greater than zero")
        if step_m <= 0.0:
            raise ValueError("step_m must be greater than zero")
        results: list[WadingResult] = []
        steps = int(round(max_depth_m / step_m))
        for index in range(steps + 1):
            depth = index * step_m
            results.append(evaluate_depth(depth, 0.0, self.config))
        return results
