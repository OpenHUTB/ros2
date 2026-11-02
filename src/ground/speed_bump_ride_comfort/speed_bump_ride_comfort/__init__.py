"""减速带通过平顺性评估：四分之一车模型、指标与建议车速。

模块可在没有 ROS 2 的环境下单独运行（``python main.py demo``），ROS 2 节点只
负责把同一套计算接到话题上，见 ``ros_node.py``。
"""

from .analysis import (
    DEFAULT_LIMIT_MPS2,
    DEFAULT_SPEEDS_MPS,
    DEFAULT_SWEEP,
    build_demo,
    recommend_max_speed,
    speed_curve,
    sweep,
)
from .bump import SpeedBump
from .comfort import (
    analysis_window,
    passage_metrics,
    peak_abs,
    rms,
    vibration_dose_value,
)
from .vehicle import GRAVITY, Passage, QuarterCar

__all__ = [
    "DEFAULT_LIMIT_MPS2",
    "DEFAULT_SPEEDS_MPS",
    "DEFAULT_SWEEP",
    "GRAVITY",
    "Passage",
    "QuarterCar",
    "SpeedBump",
    "analysis_window",
    "build_demo",
    "passage_metrics",
    "peak_abs",
    "recommend_max_speed",
    "rms",
    "speed_curve",
    "sweep",
    "vibration_dose_value",
]
