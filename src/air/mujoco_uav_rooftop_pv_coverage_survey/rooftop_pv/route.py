"""Lawnmower coverage-route generation / 往复条带覆盖航线生成。"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class RouteConfig:
    """Coverage-route parameters in metres and seconds / 米、秒制航线参数。"""

    roof_width: float = 20.0
    roof_depth: float = 14.0
    altitude: float = 8.0
    strip_spacing: float = 2.0
    speed: float = 2.0
    edge_margin: float = 1.0

    def validate(self) -> None:
        """Reject unsafe or degenerate route geometry / 拒绝不安全或退化的航线几何。"""
        positive = {
            "roof_width": self.roof_width,
            "roof_depth": self.roof_depth,
            "altitude": self.altitude,
            "strip_spacing": self.strip_spacing,
            "speed": self.speed,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive / 必须为有限正数")
        if self.edge_margin < 0.0 or not math.isfinite(self.edge_margin):
            raise ValueError("edge_margin must be finite and non-negative / 必须为有限非负数")
        if 2.0 * self.edge_margin >= min(self.roof_width, self.roof_depth):
            raise ValueError("edge_margin leaves no survey area / 边缘余量使测量区域为空")


@dataclass(frozen=True)
class Waypoint:
    """A timed coverage waypoint / 带时间的覆盖航点。"""

    index: int
    x: float
    y: float
    z: float
    yaw: float
    speed: float
    arrival_time: float
    strip_index: int
    is_turn: bool


def _strip_positions(y_min: float, y_max: float, spacing: float) -> list[float]:
    span = y_max - y_min
    intervals = max(1, math.ceil(span / spacing))
    actual_spacing = span / intervals
    return [y_min + index * actual_spacing for index in range(intervals + 1)]


def generate_lawnmower_route(config: RouteConfig) -> list[Waypoint]:
    """Generate an in-bounds boustrophedon route / 生成屋顶边界内的往复条带航线。"""
    config.validate()
    x_min = -config.roof_width / 2.0 + config.edge_margin
    x_max = config.roof_width / 2.0 - config.edge_margin
    y_min = -config.roof_depth / 2.0 + config.edge_margin
    y_max = config.roof_depth / 2.0 - config.edge_margin
    positions = _strip_positions(y_min, y_max, config.strip_spacing)

    points: list[Waypoint] = []
    elapsed = 0.0
    previous: tuple[float, float] | None = None
    for strip_index, y_value in enumerate(positions):
        endpoints = (x_min, x_max) if strip_index % 2 == 0 else (x_max, x_min)
        yaw = 0.0 if strip_index % 2 == 0 else math.pi
        for endpoint_index, x_value in enumerate(endpoints):
            if previous is not None:
                elapsed += math.hypot(x_value - previous[0], y_value - previous[1]) / config.speed
            points.append(
                Waypoint(
                    index=len(points),
                    x=x_value,
                    y=y_value,
                    z=config.altitude,
                    yaw=yaw,
                    speed=config.speed,
                    arrival_time=elapsed,
                    strip_index=strip_index,
                    is_turn=endpoint_index == 1,
                )
            )
            previous = (x_value, y_value)
    return points


def route_length(route: Iterable[Waypoint]) -> float:
    """Return planar route length / 返回平面航线长度。"""
    points = list(route)
    return sum(
        math.hypot(current.x - previous.x, current.y - previous.y)
        for previous, current in zip(points, points[1:])
    )


def save_route(route: Iterable[Waypoint], config: RouteConfig, path: Path) -> None:
    """Write a reproducible JSON route artifact / 写出可复现的 JSON 航线文件。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema_version": 1,
        "coordinate_frame": "roof_enu",
        "parameters": asdict(config),
        "waypoints": [asdict(point) for point in route],
    }
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
