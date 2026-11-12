"""故障脚本：描述某个时刻之后，每个桨还剩多少推力权限、以及有没有固定偏差。

控制侧只关心两件事：

* :meth:`FaultPlan.gains` —— ``g_i(t) ∈ [0, 1]``，第 i 个桨实际推力 = ``g_i·u_i + b_i``；
* :meth:`FaultPlan.bias` —— ``b_i(t)``，卡死时那个不受指令控制、恒定输出的推力。

真实推力与指令的对应关系与仓库里执行器故障注入模块（``uav_actuator_fault_sim``）
的约定完全一致，所以两者的实验数据可以互相解释。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

#: 支持的故障类型
FAULT_KINDS = ("healthy", "loss", "degradation", "stuck", "dual_loss",
               "progressive")
#: 中文名，给日志和表格用
FAULT_CN = {
    "healthy": "健康",
    "loss": "单桨完全失效",
    "degradation": "单桨推力衰减",
    "stuck": "单桨卡死",
    "dual_loss": "双桨失效",
    "progressive": "渐进退化",
}
#: 渐进退化的增益地板：掉到 5% 就当作彻底失效
GAIN_FLOOR = 0.05


@dataclass(frozen=True)
class FaultPlan:
    """一次飞行的故障脚本。"""

    kind: str = "healthy"
    rotor: int = 0
    rotor2: int = 1
    onset: float = 2.0
    #: loss/degradation/stuck 是"掉到多少"，progressive 是"每秒掉多少"
    severity: float = 1.0
    ramp: float = 0.3

    def __post_init__(self) -> None:
        if self.kind not in FAULT_KINDS:
            raise ValueError(f"未知故障类型 {self.kind!r}，可选：{FAULT_KINDS}")
        for index in (self.rotor, self.rotor2):
            if not 0 <= index < 4:
                raise ValueError(f"桨序号必须在 0..3 之间，收到 {index}")
        if self.onset < 0.0:
            raise ValueError("onset 不能为负")
        if self.ramp < 0.0:
            raise ValueError("ramp 不能为负")
        if self.severity < 0.0:
            raise ValueError("severity 不能为负")
        if self.kind in ("loss", "degradation", "stuck") and not 0.0 <= self.severity <= 1.0:
            raise ValueError("severity 表示损失比例，必须在 [0, 1] 之间")
        if self.kind == "progressive" and self.severity <= 0.0:
            raise ValueError("progressive 的 severity 表示每秒衰减量，必须 > 0")
        if self.kind == "dual_loss" and self.rotor == self.rotor2:
            raise ValueError("双桨失效需要两个不同的桨")

    # -- 内部 -------------------------------------------------------------
    def alpha(self, t: float) -> float:
        """故障过渡系数：onset 之前 0，ramp 之后 1。"""
        if t < self.onset:
            return 0.0
        if self.ramp <= 0.0:
            return 1.0
        return float(min(1.0, (t - self.onset) / self.ramp))

    # -- 对外 -------------------------------------------------------------
    def gains(self, t: float) -> np.ndarray:
        """四个桨的推力权限。"""
        g = np.ones(4)
        a = self.alpha(t)
        if a <= 0.0 or self.kind == "healthy":
            return g
        if self.kind == "loss":
            g[self.rotor] = 1.0 - a
        elif self.kind == "degradation":
            g[self.rotor] = 1.0 - self.severity * a
        elif self.kind == "stuck":
            g[self.rotor] = 1.0 - a
        elif self.kind == "dual_loss":
            g[self.rotor] = 1.0 - a
            g[self.rotor2] = 1.0 - a
        elif self.kind == "progressive":
            decay = self.severity * max(0.0, t - self.onset)
            g[self.rotor] = max(GAIN_FLOOR, 1.0 - decay)
        return g

    def bias(self, t: float, hover_thrust: float) -> np.ndarray:
        """卡死桨的固定输出（其余故障为 0）。"""
        b = np.zeros(4)
        if self.kind == "stuck":
            b[self.rotor] = hover_thrust * self.alpha(t)
        return b

    def onset_time(self) -> float:
        """任务段结束、开始迫降的时刻。

        ``healthy`` 也返回 ``onset``：那时没有故障要注入，但这个时刻仍然是
        "从悬停转到迫降"的分界点（节点和评测都用它切两个阶段）。
        """
        return float(self.onset)

    def label(self) -> str:
        if self.kind == "healthy":
            return FAULT_CN["healthy"]
        text = f"{FAULT_CN[self.kind]}@桨{self.rotor}"
        if self.kind == "dual_loss":
            text += f"+桨{self.rotor2}"
        # 表格里同名工况容易看着像重复行，把剩余推力一起写进标签
        if self.kind in ("loss", "degradation", "stuck"):
            text += f"（剩余推力{100.0 * (1.0 - self.severity):.0f}%）"
        elif self.kind == "progressive":
            text += f"（每秒衰减{self.severity:.2f}）"
        return text


#: 默认对比用的故障脚本（覆盖单桨、双桨、渐变与卡死）
DEFAULT_PLANS = (
    FaultPlan(kind="healthy"),
    FaultPlan(kind="loss", rotor=0, onset=2.0),
    FaultPlan(kind="degradation", rotor=1, onset=2.0, severity=0.6),
    FaultPlan(kind="stuck", rotor=2, onset=2.0),
    FaultPlan(kind="dual_loss", rotor=0, rotor2=2, onset=2.0),
    FaultPlan(kind="progressive", rotor=3, onset=1.5, severity=0.4, ramp=0.0),
)
