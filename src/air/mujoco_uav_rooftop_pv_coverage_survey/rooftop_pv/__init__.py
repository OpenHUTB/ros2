"""Urban rooftop photovoltaic survey primitives.

城市屋顶光伏测量基础组件。
"""

from .route import RouteConfig, Waypoint, generate_lawnmower_route
from .scene import SCENARIOS, load_model, scenario_path

__all__ = [
    "RouteConfig",
    "SCENARIOS",
    "Waypoint",
    "generate_lawnmower_route",
    "load_model",
    "scenario_path",
]
