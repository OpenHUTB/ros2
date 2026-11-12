"""无人机桨故障下的端到端深度图推力策略。

模块边界：

* ``obs`` / ``env`` / ``expert`` / ``faults`` / ``allocation`` —— 数据与仿真侧，只用 numpy + mujoco；
* ``policy`` / ``train`` —— 神经网络侧，延迟导入 torch；
* ``data`` / ``evaluate`` / ``render`` / ``cli`` / ``ros_node`` —— 流水线与部署。

顶层只导出不依赖 torch / mujoco 的那部分，保证 ``import`` 本包在任何机器上都能成功。
"""

from .evaluate import FlightRow, aggregate, flight_table, run_episode
from .faults import FAULT_KINDS, FaultPlan
from .obs import DEPTH_SIZE, OBS_DIM, PROPRIO_DIM, build_observation, crop_depth
from .expert import TaskConfig, ThrustExpert

__version__ = "1.0.0"

__all__ = [
    "DEPTH_SIZE",
    "FAULT_KINDS",
    "FlightRow",
    "OBS_DIM",
    "PROPRIO_DIM",
    "FaultPlan",
    "TaskConfig",
    "ThrustExpert",
    "__version__",
    "aggregate",
    "build_observation",
    "crop_depth",
    "flight_table",
    "run_episode",
]
