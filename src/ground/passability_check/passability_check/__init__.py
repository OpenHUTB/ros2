"""passability_check —— 车辆几何通过性核算（接近角 / 离去角 / 纵向通过角 / 托底）。

模块只依赖 Python 标准库：`math` / `json` / `io` / `os`，不 import rclpy，
因此可以在没有 ROS、没有仿真器、没有显示器的机器上直接跑。
"""

__version__ = "0.1.0"

from .vehicle import VehicleSpec, VehicleError, PRESET_VEHICLES, get_vehicle
from .terrain import Terrain, TerrainError
from .clearance import (
    ClearanceError,
    contact_front_x,
    seat,
    lower_contour_world,
    penetration_at,
    scan_profile,
)
from .scenario import Scenario, ScenarioError, load_scenario, list_scenarios
from .report import (
    ASSERT_NAMES,
    build_assertions,
    all_passed,
    evaluate,
    format_report,
    render_profile,
    scenario_matrix_csv,
    to_json_payload,
)

__all__ = [
    "__version__",
    "VehicleSpec",
    "VehicleError",
    "PRESET_VEHICLES",
    "get_vehicle",
    "Terrain",
    "TerrainError",
    "ClearanceError",
    "contact_front_x",
    "seat",
    "lower_contour_world",
    "penetration_at",
    "scan_profile",
    "Scenario",
    "ScenarioError",
    "load_scenario",
    "list_scenarios",
    "ASSERT_NAMES",
    "build_assertions",
    "all_passed",
    "evaluate",
    "format_report",
    "render_profile",
    "scenario_matrix_csv",
    "to_json_payload",
]
