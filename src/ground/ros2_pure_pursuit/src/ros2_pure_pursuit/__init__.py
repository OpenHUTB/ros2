"""ros2_pure_pursuit：差速机器人纯追踪路径跟踪 + 激光反应式避障。"""

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
]

__version__ = "1.0.0"
