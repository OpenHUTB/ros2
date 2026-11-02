# -*- coding: utf-8 -*-
"""执行器（旋翼）故障模型。

设计要点
--------
1. **不侵入底座**。:class:`FaultInjector` 只作用在"指令推力"这一层：
   控制器 ``GeometricController.act()`` 算出 4 维推力后，先经 :meth:`FaultInjector.apply`
   映射成该桨**实际能产生**的推力，再交给 ``Quadrotor.apply_rotor_thrust()``。
   因此 ``uav_mujoco_sim`` 一行都不用改。
2. **统一用增益 + 偏移描述**。任意故障都在每个桨上折成一对量：

   .. math::

       T_i^{act}(t) = g_i(t)\\, u_i^{cmd}(t) + b_i(t)

   - 完全失效 ``loss``       : :math:`g=0,\\ b=0`
   - 部分失效 ``degradation``: :math:`g=\\lambda,\\ b=0`
   - 渐进衰减 ``progressive``: :math:`g=\\lambda(t),\\ b=0`
   - 卡滞     ``stuck``      : :math:`g=0,\\ b=\\lambda T_{max}`

   这样故障检测网络（PR2）只需要回归一个 4 维量 :math:`\\hat g`，标签定义无歧义；
   容错控制（PR3）也只需要 :math:`\\mathrm{diag}(g)` 就能重建控制效率矩阵。
3. **可重复**。同一个 ``FaultScenario`` + 同一个 ``seed`` 必须产生逐比特一致的轨迹，
   否则指标没法比较。
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

#: 旋翼数量（与 uav_mujoco_sim 的 QuadParams 一致）
ROTOR_COUNT = 4

#: 支持的故障类型
FAULT_KINDS: Tuple[str, ...] = ("loss", "degradation", "stuck", "progressive")

#: 各故障类型的中文名（文档与日志用）
FAULT_CN: Dict[str, str] = {
    "loss": "完全失效",
    "degradation": "部分失效",
    "stuck": "卡滞",
    "progressive": "渐进衰减",
}


@dataclass(frozen=True)
class RotorFault:
    """单个旋翼的一条故障记录。

    Parameters
    ----------
    rotor:
        旋翼编号 0..3。底座 ``mixer()`` 的输出顺序即此编号。
    kind:
        ``loss`` / ``degradation`` / ``stuck`` / ``progressive``。
    onset:
        故障注入时刻，单位秒（相对一次飞行开始）。
    severity:
        - ``degradation``：稳态健康因子 :math:`\\lambda \\in [0, 1]`，越小越严重；
        - ``stuck``：卡滞推力占单桨最大推力的比例 :math:`\\in [0, 1]`；
        - ``progressive``：衰减速率 :math:`-\\dot\\lambda`，单位 1/s；
        - ``loss``：忽略。
    ramp:
        从健康到故障的过渡时间，单位秒。0 表示瞬时跳变（最恶劣，也是默认）。
    """

    rotor: int
    kind: str
    onset: float = 2.0
    severity: float = 0.0
    ramp: float = 0.0

    def __post_init__(self) -> None:
        if not isinstance(self.rotor, int) or not 0 <= self.rotor < ROTOR_COUNT:
            raise ValueError(f"rotor 必须是 0..{ROTOR_COUNT - 1} 的整数，收到 {self.rotor!r}")
        if self.kind not in FAULT_KINDS:
            raise ValueError(f"kind 必须是 {FAULT_KINDS} 之一，收到 {self.kind!r}")
        if self.onset < 0.0:
            raise ValueError(f"onset 不能为负，收到 {self.onset}")
        if self.ramp < 0.0:
            raise ValueError(f"ramp 不能为负，收到 {self.ramp}")
        if self.kind == "degradation" and not 0.0 <= self.severity <= 1.0:
            raise ValueError(f"degradation 的 severity 必须在 [0,1]，收到 {self.severity}")
        if self.kind == "stuck" and not 0.0 <= self.severity <= 1.0:
            raise ValueError(f"stuck 的 severity 必须在 [0,1]，收到 {self.severity}")
        if self.kind == "progressive" and self.severity <= 0.0:
            raise ValueError(f"progressive 的 severity（衰减速率）必须为正，收到 {self.severity}")

    # ------------------------------------------------------------------ #
    def gain(self, t: float, max_thrust: float) -> float:
        """该桨在时刻 ``t`` 的推力增益 :math:`g_i(t)`。"""
        if t < self.onset:
            return 1.0
        # ramp 内线性过渡，ramp=0 时 alpha 直接为 1
        alpha = 1.0 if self.ramp <= 0.0 else min(1.0, (t - self.onset) / self.ramp)
        if self.kind == "loss":
            return 1.0 - alpha
        if self.kind == "degradation":
            return 1.0 + (self.severity - 1.0) * alpha
        if self.kind == "stuck":
            # 卡滞：不再响应指令，增益归零，推力由 offset 提供
            return 1.0 - alpha
        if self.kind == "progressive":
            # 从现在开始以 severity 的速率线性衰减，下限 0.05（保留一点残余推力）
            return max(0.05, 1.0 - self.severity * (t - self.onset))
        raise AssertionError(f"未处理的故障类型 {self.kind}")  # pragma: no cover

    def offset(self, t: float, max_thrust: float) -> float:
        """该桨在时刻 ``t`` 的推力偏移 :math:`b_i(t)`（仅 ``stuck`` 非零）。"""
        if self.kind != "stuck" or t < self.onset:
            return 0.0
        alpha = 1.0 if self.ramp <= 0.0 else min(1.0, (t - self.onset) / self.ramp)
        return self.severity * max_thrust * alpha

    def label(self) -> str:
        return f"{FAULT_CN[self.kind]}@桨{self.rotor}"


@dataclass(frozen=True)
class SensorFault:
    """传感器故障（只影响感知链路，不影响动力学）。

    PR2（故障检测网络）用它做"传感器侧"故障类别，和旋翼故障区分开。
    """

    #: IMU 陀螺零偏，rad/s
    gyro_bias: float = 0.0
    #: IMU 噪声放大倍数（1.0 = 不放大）
    noise_scale: float = 1.0
    #: 采样丢包概率
    dropout: float = 0.0

    def __post_init__(self) -> None:
        if self.noise_scale <= 0.0:
            raise ValueError(f"noise_scale 必须为正，收到 {self.noise_scale}")
        if not 0.0 <= self.dropout <= 1.0:
            raise ValueError(f"dropout 必须在 [0,1]，收到 {self.dropout}")

    @property
    def active(self) -> bool:
        return bool(self.gyro_bias or self.noise_scale != 1.0 or self.dropout)


@dataclass(frozen=True)
class FaultSpec:
    """一次飞行要注入的完整故障集合。"""

    rotor_faults: Tuple[RotorFault, ...] = ()
    sensor: SensorFault = SensorFault()

    def __post_init__(self) -> None:
        seen = set()
        for f in self.rotor_faults:
            if f.rotor in seen:
                raise ValueError(f"同一旋翼注入了多条故障：rotor={f.rotor}")
            seen.add(f.rotor)

    @property
    def earliest_onset(self) -> Optional[float]:
        """最早一条故障的注入时刻；无故障返回 ``None``。"""
        return min((f.onset for f in self.rotor_faults), default=None)

    def describe(self) -> str:
        if not self.rotor_faults:
            base = "无旋翼故障"
        else:
            base = "、".join(f.label() for f in self.rotor_faults)
        if self.sensor.active:
            base += f"；传感器故障(陀螺零偏{self.sensor.gyro_bias:+.3f} rad/s)"
        return base


class FaultInjector:
    """把指令推力映射成实际推力的故障注入器。

    用法::

        inj = FaultInjector(spec, max_thrust=6.0)
        thrusts = ctrl.act(quad, p_d, v_d, a_d, yaw_d)   # 控制器算出的 4 维推力
        quad.apply_rotor_thrust(inj.apply(thrusts, t))   # 故障后的实际推力
    """

    def __init__(self, spec: FaultSpec, max_thrust: float = 6.0) -> None:
        self.spec = spec
        self.max_thrust = float(max_thrust)
        # 逐桨预排序，apply() 里避免重复遍历
        self._by_rotor: List[List[RotorFault]] = [[] for _ in range(ROTOR_COUNT)]
        for f in spec.rotor_faults:
            self._by_rotor[f.rotor].append(f)

    # ------------------------------------------------------------------ #
    def gains(self, t: float) -> np.ndarray:
        """当前时刻 4 个桨的增益 :math:`g_i`（检测网络的回归标签）。"""
        g = np.ones(ROTOR_COUNT, dtype=float)
        for i, faults in enumerate(self._by_rotor):
            for f in faults:
                g[i] *= f.gain(t, self.max_thrust)
        return np.clip(g, 0.0, 1.0)

    def offsets(self, t: float) -> np.ndarray:
        """当前时刻 4 个桨的推力偏移 :math:`b_i`。"""
        b = np.zeros(ROTOR_COUNT, dtype=float)
        for i, faults in enumerate(self._by_rotor):
            for f in faults:
                b[i] += f.offset(t, self.max_thrust)
        return b

    def apply(self, thrusts: Sequence[float], t: float) -> np.ndarray:
        """``thrusts``（4 维指令推力）→ 故障后的实际推力，裁剪到 ``[0, max_thrust]``。"""
        u = np.asarray(thrusts, dtype=float).reshape(ROTOR_COUNT)
        out = self.gains(t) * u + self.offsets(t)
        return np.clip(out, 0.0, self.max_thrust)

    def active(self, t: float) -> bool:
        """是否已有故障处于生效状态。"""
        onset = self.spec.earliest_onset
        return onset is not None and t >= onset

    def worst_case_gain(self) -> float:
        """从开始到永久：所有桨中可能出现的最小增益（用于快速判"是否救得回来"）。"""
        g_min = 1.0
        for f in self.spec.rotor_faults:
            if f.kind == "loss":
                g_min = min(g_min, 0.0)
            elif f.kind == "degradation":
                g_min = min(g_min, f.severity)
            elif f.kind == "progressive":
                g_min = min(g_min, 0.05)
        return g_min


# --------------------------------------------------------------------------- #
# 故障场景库
# --------------------------------------------------------------------------- #
def single_rotor_loss(rotor: int = 0, onset: float = 2.0) -> FaultSpec:
    """单桨完全失效 —— 本模块的核心演示场景，实测 0.64 s 坠地。"""
    return FaultSpec(rotor_faults=(RotorFault(rotor, "loss", onset),))


def dual_rotor_loss(first: int = 0, second: int = 2, onset: float = 2.0) -> FaultSpec:
    """对角双桨完全失效 —— 极端场景，用来考察容错控制的可控性边界。"""
    return FaultSpec(rotor_faults=(
        RotorFault(first, "loss", onset),
        RotorFault(second, "loss", onset),
    ))


def partial_degradation(rotor: int = 1, severity: float = 0.4, onset: float = 2.0) -> FaultSpec:
    """单桨推力掉到额定值的 40% —— 能飞，但性能明显退化。"""
    return FaultSpec(rotor_faults=(RotorFault(rotor, "degradation", onset, severity),))


def rotor_stuck(rotor: int = 3, severity: float = 0.6, onset: float = 2.0) -> FaultSpec:
    """单桨卡滞在 60% 推力 —— 持续产生偏航力矩，且完全不响应指令。"""
    return FaultSpec(rotor_faults=(RotorFault(rotor, "stuck", onset, severity),))


def progressive_decay(rotor: int = 2, rate: float = 0.25, onset: float = 2.0) -> FaultSpec:
    """单桨推力以 0.25/s 线性衰减 —— 检测难度最高，需要时序记忆。"""
    return FaultSpec(rotor_faults=(RotorFault(rotor, "progressive", onset, rate),))


def sensor_gyro_bias(bias: float = 0.05) -> FaultSpec:
    """仅 IMU 陀螺零偏，旋翼健康 —— PR2 的传感器故障负样本。"""
    return FaultSpec(sensor=SensorFault(gyro_bias=bias))


def combine(*specs: FaultSpec) -> FaultSpec:
    """把多个 ``FaultSpec`` 的旋翼故障合并（传感器故障取最后一个非默认值）。"""
    faults: List[RotorFault] = []
    sensor = SensorFault()
    for s in specs:
        faults.extend(s.rotor_faults)
        if s.sensor.active:
            sensor = s.sensor
    # 同桨重复时后者覆盖前者，避免 __post_init__ 报错
    uniq: Dict[int, RotorFault] = {}
    for f in faults:
        uniq[f.rotor] = replace(f)
    return FaultSpec(rotor_faults=tuple(uniq.values()), sensor=sensor)
