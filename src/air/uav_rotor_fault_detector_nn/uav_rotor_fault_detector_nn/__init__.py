"""uav_rotor_fault_detector_nn —— 四旋翼桨故障的健康度估计（神经网络）。

模块定位对应课程任务②（传感器感知 + 运动控制里的"感知"部分）：把机载 IMU 与
控制器指令喂给一个 1D-CNN + GRU 网络，实时估计四个桨各自还剩多少推力权限，
为后续容错控制提供状态输入。

分层的原则是"轻的东西放前面、重的东西放后面"：

* ``features`` / ``data`` / ``metrics`` —— 只依赖 numpy，导入即用；
* ``model`` / ``train`` / ``infer`` —— 依赖 torch，走 ``__getattr__`` 惰性导入；
* ``ros_node`` —— 依赖 rclpy，只在 ROS 环境里导入。

这样在没有 torch 的机器上也能 ``import uav_rotor_fault_detector_nn`` 去用故障场景定义。
"""

from __future__ import annotations

__version__ = "1.0.0"

from .features import (
    FEATURE_DIM,
    FEATURE_NAMES,
    FEATURE_SCALE,
    ROTOR_COUNT,
    build_step_features,
    make_windows,
    quat_to_matrix,
    specific_force,
)
from .data import (
    FAULT_CN,
    FAULT_KINDS,
    PHYSICAL_SCENES,
    ROTOR_FAULT_KINDS,
    SENSOR_FAULT_KINDS,
    Episode,
    FaultPlan,
    build_dataset,
    load_dataset,
    random_plan,
    save_dataset,
    simulate_episode,
    split_by_episode,
)
from .metrics import (
    DETECT_GAIN_THRESHOLD,
    FAULT_GAIN_THRESHOLD,
    confusion,
    detection_latency,
    mean_absolute_error,
    per_rotor_mae,
    summarize,
)

__all__ = [
    "__version__",
    # features
    "FEATURE_DIM", "FEATURE_NAMES", "FEATURE_SCALE", "ROTOR_COUNT",
    "build_step_features", "make_windows", "quat_to_matrix", "specific_force",
    # data
    "FAULT_CN", "FAULT_KINDS", "PHYSICAL_SCENES", "ROTOR_FAULT_KINDS",
    "SENSOR_FAULT_KINDS", "Episode", "FaultPlan", "build_dataset",
    "load_dataset", "random_plan", "save_dataset", "simulate_episode",
    "split_by_episode",
    # metrics
    "DETECT_GAIN_THRESHOLD", "FAULT_GAIN_THRESHOLD", "confusion",
    "detection_latency", "mean_absolute_error", "per_rotor_mae", "summarize",
    # 惰性导入
    "FaultDetectorNet", "build_model", "load_checkpoint", "save_checkpoint",
    "TrainConfig", "train_detector", "HealthEstimator",
]

#: 重依赖（torch / rclpy）的名字 -> 所在子模块
_LAZY = {
    "FaultDetectorNet": "model",
    "build_model": "model",
    "load_checkpoint": "model",
    "save_checkpoint": "model",
    "TrainConfig": "train",
    "train_detector": "train",
    "HealthEstimator": "infer",
}


def __getattr__(name: str):
    module_name = _LAZY.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    import importlib

    module = importlib.import_module("." + module_name, __name__)
    value = getattr(module, name)
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted(set(list(globals()) + list(_LAZY)))
