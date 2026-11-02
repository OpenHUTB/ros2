"""ros2_pure_pursuit：差速机器人纯追踪路径跟踪 + 激光反应式避障。"""

from .arbitrate import ArbitrationResult, arbitrate
from .obstacle import AvoidInfo, ObstacleAvoider
from .paths import Path, circle, figure_eight, s_curve, straight
from .pure_pursuit import ControlInfo, PurePursuitController
from .robot import DiffDriveRobot, Pose

__all__ = [
    "Path",
    "circle",
    "figure_eight",
    "s_curve",
    "straight",
    "ControlInfo",
    "PurePursuitController",
    "DiffDriveRobot",
    "Pose",
    "AvoidInfo",
    "ObstacleAvoider",
    "arbitrate",
    "ArbitrationResult",
]

__version__ = "1.0.0"
