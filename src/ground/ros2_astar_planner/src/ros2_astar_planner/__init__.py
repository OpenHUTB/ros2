"""ROS2 栅格全局路径规划（A*）核心算法包。

仅依赖 NumPy，可在无 ROS 环境下运行离线仿真与单元测试。
"""

from .astar import AStarConfig, AStarPlanner, PlanResult, count_turns, path_length, path_world
from .gridmap import GridMap, GridSpec, corridor_map, maze_map, random_obstacle_map
from .smoothing import (
    count_turns_world,
    gradient_smooth,
    polyline_length,
    segment_is_free,
    shortcut,
    smooth_path,
)
from .trajectory import Trajectory, check_acceleration, compute_curvature, sample_trajectory, time_parameterize

__all__ = [
    "AStarConfig",
    "AStarPlanner",
    "PlanResult",
    "path_length",
    "path_world",
    "count_turns",
    "GridMap",
    "GridSpec",
    "corridor_map",
    "maze_map",
    "random_obstacle_map",
    "segment_is_free",
    "shortcut",
    "gradient_smooth",
    "smooth_path",
    "polyline_length",
    "count_turns_world",
    "time_parameterize",
    "sample_trajectory",
    "compute_curvature",
    "check_acceleration",
    "Trajectory",
]

__version__ = "1.0.0"
