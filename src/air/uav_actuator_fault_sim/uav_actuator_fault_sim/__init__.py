# -*- coding: utf-8 -*-
"""``uav_actuator_fault_sim``：无人机执行器/传感器故障注入仿真层。

本包是「神经容错飞行」系列的第一个模块，负责**在仿真里真的把桨打坏**，
并给出可比较的飞行指标。后续的故障检测网络、容错控制器、应急降落策略
都以本包生成的飞行数据为训练与评测基准。

模块划分
--------
``fault``
    故障数学模型。所有故障统一折算成 ``T_act = g(t)·u_cmd + b(t)`` 的
    增益/偏置形式，四种类型（完全失效、推力衰减、卡死、渐进退化）共用一套算子。
``scenario``
    7 种故障场景 × 5 种物理场景 × 随机种子的用例矩阵。
``metrics``
    坠机判定、生存率、触地速度、位置 RMSE、控制能量等指标。
``runner``
    把「场景 + 故障 + 控制器」跑成一条轨迹。
``keyboard_teleop``
    键盘直飞（任务①要求的遥操作入口）。
``ros_node``
    ROS2 节点，订阅 ``/uav/cmd_vel`` 并把故障状态发到 ``/uav/fault``。

用法
----
命令行走 ``main.py``：:

    python main.py --demo            # 不开 ROS，直接跑对比表
    python main.py --eval            # 生成 results/comparison.csv
    python main.py --gif out.gif     # 录故障对比动图

注意：本包**不 import mujoco**，只有 ``runner`` 真正用到时才通过 ``_compat`` 加载底座，
因此 ``import uav_actuator_fault_sim`` 在没有 MuJoCo 的环境里也不会炸。
"""

__version__ = "1.0.0"

from .fault import (  # noqa: F401
    FAULT_KINDS,
    ROTOR_COUNT,
    FaultInjector,
    FaultSpec,
    RotorFault,
    SensorFault,
    combine,
    dual_rotor_loss,
    partial_degradation,
    progressive_decay,
    rotor_stuck,
    sensor_gyro_bias,
    single_rotor_loss,
)
from .metrics import FlightResult, aggregate, group_summary  # noqa: F401
from .scenario import (  # noqa: F401
    FAULT_SCENARIOS,
    PHYSICAL_SCENES,
    FlightCase,
    full_matrix,
    make_case,
    training_matrix,
)

__all__ = [
    "__version__",
    # fault
    "FAULT_KINDS", "ROTOR_COUNT", "FaultInjector", "FaultSpec", "RotorFault",
    "SensorFault", "combine", "dual_rotor_loss", "partial_degradation",
    "progressive_decay", "rotor_stuck", "sensor_gyro_bias", "single_rotor_loss",
    # metrics
    "FlightResult", "aggregate", "group_summary",
    # scenario
    "FAULT_SCENARIOS", "PHYSICAL_SCENES", "FlightCase", "full_matrix",
    "make_case", "training_matrix",
]
