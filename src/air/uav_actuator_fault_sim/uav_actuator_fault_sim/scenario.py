# -*- coding: utf-8 -*-
"""故障场景库：把"故障种类"和底座的"物理场景"交叉成可枚举的实验矩阵。

约定
----
* **物理场景**（由 ``uav_mujoco_sim`` 提供，这里只按名字引用，不复制 XML）：
  ``open_ground`` / ``obstacle_field`` / ``corridor`` / ``gate_track`` / ``urban_block``
* **故障场景**（本模块定义）：6 种，见 :data:`FAULT_SCENARIOS`
* 一次"实验"= (物理场景, 故障场景, seed)，共 30 种组合，可全自动批跑。
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from .fault import (
    FAULT_CN,
    FaultSpec,
    RotorFault,
    SensorFault,
    dual_rotor_loss,
    partial_degradation,
    progressive_decay,
    rotor_stuck,
    sensor_gyro_bias,
    single_rotor_loss,
)

#: 底座 ``uav_mujoco_sim`` 提供的 5 个物理场景
PHYSICAL_SCENES: Tuple[str, ...] = (
    "open_ground",
    "obstacle_field",
    "corridor",
    "gate_track",
    "urban_block",
)

#: 每个物理场景在批跑时的默认飞行时长（秒）；urban_block 场景大，需要久一点
SCENE_DURATION: Dict[str, float] = {
    "open_ground": 8.0,
    "obstacle_field": 8.0,
    "corridor": 8.0,
    "gate_track": 10.0,
    "urban_block": 10.0,
}


@dataclass(frozen=True)
class FaultScenario:
    """一个具名的故障场景。"""

    name: str
    #: 人类可读的一句说明（会出现在 README 与文档的表格里）
    description: str
    #: 构造 ``FaultSpec`` 的工厂：``make(rotor, onset, severity) -> FaultSpec``
    make: object
    #: 该场景是否有学习型方法发挥空间（决定它是否进入 PR2/PR3 的训练集）
    learnable: bool = True

    def build(self, rotor: int = 0, onset: float = 2.0,
              severity: Optional[float] = None) -> FaultSpec:
        """按场景生成具体故障，``rotor``/``onset`` 可随机化以扩充训练集。

        多数场景的工厂函数不接受 ``severity``（严重度由场景本身固定），
        此时自动退回两参数调用，避免调用方还要记住每个场景的签名。
        """
        if severity is None:
            return self.make(rotor=rotor, onset=onset)
        try:
            return self.make(rotor=rotor, onset=onset, severity=severity)
        except TypeError:
            return self.make(rotor=rotor, onset=onset)

    def label(self) -> str:
        return self.name


#: 6 个具名故障场景。``make`` 的签名统一为关键字参数，便于 :meth:`FaultScenario.build` 随机化。
FAULT_SCENARIOS: Dict[str, FaultScenario] = {
    "single_loss": FaultScenario(
        name="single_loss",
        description="单桨完全失效：推力归零，几何控制器 0.64 s 内翻转坠地",
        make=lambda rotor=0, onset=2.0: single_rotor_loss(rotor, onset),
        learnable=True,
    ),
    "dual_loss": FaultScenario(
        name="dual_loss",
        description="对角双桨完全失效：单靠重分配无法维持，考察可控性边界",
        make=lambda rotor=0, onset=2.0: dual_rotor_loss(rotor, (rotor + 2) % 4, onset),
        learnable=True,
    ),
    "partial_degradation": FaultScenario(
        name="partial_degradation",
        description="单桨推力衰减到 40%：能飞但性能退化，适合检验检测器灵敏度",
        make=lambda rotor=1, onset=2.0, severity=0.4: partial_degradation(rotor, severity, onset),
        learnable=True,
    ),
    "stuck": FaultScenario(
        name="stuck",
        description="单桨卡滞在 60% 推力：不响应指令且持续产生偏航力矩",
        make=lambda rotor=3, onset=2.0, severity=0.6: rotor_stuck(rotor, severity, onset),
        learnable=True,
    ),
    "progressive": FaultScenario(
        name="progressive",
        description="单桨推力以 0.25/s 线性衰减：渐变故障，必须靠时序记忆才能及早发现",
        make=lambda rotor=2, onset=2.0, severity=0.25: progressive_decay(rotor, severity, onset),
        learnable=True,
    ),
    "sensor_bias": FaultScenario(
        name="sensor_bias",
        description="仅 IMU 陀螺零偏 +0.05 rad/s，旋翼全部健康：传感器侧故障的对照",
        make=lambda rotor=0, onset=0.0, severity=0.05: sensor_gyro_bias(severity),
        learnable=True,
    ),
    "healthy": FaultScenario(
        name="healthy",
        description="无任何故障：所有对比实验的基准行",
        make=lambda rotor=0, onset=0.0: FaultSpec(),
        learnable=False,
    ),
}

#: 参与训练/评测的故障场景（去掉 healthy 基准）
FAULT_ONLY: Tuple[str, ...] = tuple(
    k for k, v in FAULT_SCENARIOS.items() if k != "healthy"
)


@dataclass(frozen=True)
class FlightCase:
    """一个可执行的实验单元：(物理场景, 故障场景, 随机化参数)。"""

    physical_scene: str
    fault_scenario: str
    rotor: int
    onset: float
    severity: Optional[float]
    seed: int

    def build_spec(self) -> FaultSpec:
        sc = FAULT_SCENARIOS[self.fault_scenario]
        return sc.build(rotor=self.rotor, onset=self.onset, severity=self.severity)

    def duration(self) -> float:
        return SCENE_DURATION.get(self.physical_scene, 8.0)

    def tag(self) -> str:
        sev = "" if self.severity is None else f"_s{self.severity:g}"
        return (
            f"{self.physical_scene}__{self.fault_scenario}__"
            f"r{self.rotor}_t{self.onset:g}{sev}_s{self.seed}"
        )


def make_case(physical_scene: str, fault_scenario: str, seed: int = 0,
              rotor: Optional[int] = None, onset: Optional[float] = None,
              severity: Optional[float] = None) -> FlightCase:
    """按名字建一个实验单元；未指定的量由 ``seed`` 决定。"""
    if physical_scene not in PHYSICAL_SCENES:
        raise ValueError(f"未知物理场景 {physical_scene!r}，可选 {PHYSICAL_SCENES}")
    if fault_scenario not in FAULT_SCENARIOS:
        raise ValueError(f"未知故障场景 {fault_scenario!r}，可选 {tuple(FAULT_SCENARIOS)}")
    rng = random.Random(seed)
    return FlightCase(
        physical_scene=physical_scene,
        fault_scenario=fault_scenario,
        rotor=rng.randrange(4) if rotor is None else rotor,
        onset=round(rng.uniform(1.5, 3.5), 2) if onset is None else onset,
        severity=severity,
        seed=seed,
    )


def full_matrix(seeds: Sequence[int] = (0,)) -> List[FlightCase]:
    """完整实验矩阵：5 物理场景 × 7 故障场景 × len(seeds)。

    默认 1 个 seed 时是 35 个单元；PR2/PR3 训练用 3 个 seed 时是 105 个。
    """
    cases: List[FlightCase] = []
    for seed in seeds:
        for scene in PHYSICAL_SCENES:
            for fault in FAULT_SCENARIOS:
                cases.append(make_case(scene, fault, seed=seed))
    return cases


def training_matrix(seeds: Sequence[int] = (0, 1, 2)) -> List[FlightCase]:
    """PR2/PR3 训练用矩阵：只含故障场景（不含 healthy），3 个 seed → 90 个单元。"""
    cases: List[FlightCase] = []
    for seed in seeds:
        for scene in PHYSICAL_SCENES:
            for fault in FAULT_ONLY:
                cases.append(make_case(scene, fault, seed=seed))
    return cases
