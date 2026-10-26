# -*- coding: utf-8 -*-
"""飞行指标：把一条轨迹折算成可比较的标量。

所有指标都围绕一个判断：**故障之后这台机器还能不能安全收场**。
因此核心三件是「活下来没有」「掉得多深」「落地多狠」，其余（漂移、能量、偏航转速）
用来解释"为什么"。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

#: 判为"坠地"的高度阈值，米（底座场景地面 z=0，机体中心到地面约 0.15 m）
CRASH_ALTITUDE = 0.15

#: 判为"安全着陆"的触地速度阈值，m/s
SAFE_TOUCHDOWN_SPEED = 0.5


@dataclass
class FlightResult:
    """一次仿真的完整记录。数组均按时间轴第一维排列。"""

    #: 时间戳，形状 (N,)
    t: np.ndarray
    #: 世界系位置，形状 (N, 3)
    pos: np.ndarray
    #: 世界系速度，形状 (N, 3)
    vel: np.ndarray
    #: 机体系角速度 rad/s，形状 (N, 3)
    omega: np.ndarray
    #: 欧拉角 deg，形状 (N, 3)：roll / pitch / yaw
    euler: np.ndarray
    #: 实际施加的桨推力 N，形状 (N, 4)
    thrusts: np.ndarray
    #: 故障增益真值，形状 (N, 4)；无故障时全 1
    health: np.ndarray
    #: 目标悬停点，形状 (3,)
    target: np.ndarray
    #: 故障注入时刻，秒；无故障为 None
    onset: Optional[float] = None
    #: 故障场景名，仅用于报表
    scenario: str = ""
    #: 物理场景名
    scene: str = ""
    #: 控制器下发的期望桨推力 N，形状 (N, 4)；与 ``thrusts``（故障后实际值）对照，
    #: 可算出分配器"想给"和"真给到"的差
    desired: Optional[np.ndarray] = None
    #: 采样得到的 RGB 帧（仅当 record_frames=True）
    frames: List[np.ndarray] = field(default_factory=list)

    # ------------------------------------------------------------------ #
    # 派生量
    # ------------------------------------------------------------------ #
    @property
    def altitude(self) -> np.ndarray:
        return self.pos[:, 2]

    @property
    def min_altitude(self) -> float:
        return float(self.altitude.min())

    @property
    def crashed(self) -> bool:
        return bool(self.min_altitude < CRASH_ALTITUDE)

    @property
    def crash_index(self) -> Optional[int]:
        """首次跌破坠地阈值的时间下标。"""
        idx = np.flatnonzero(self.altitude < CRASH_ALTITUDE)
        return int(idx[0]) if idx.size else None

    @property
    def crash_delay(self) -> Optional[float]:
        """从故障注入到坠地的时间，秒。无故障或未坠地返回 None。"""
        k = self.crash_index
        if k is None:
            return None
        t_crash = float(self.t[k])
        if self.onset is None:
            return t_crash
        return max(0.0, t_crash - self.onset)

    @property
    def touchdown_speed(self) -> Optional[float]:
        """触地瞬间的速度模长，m/s。未坠地返回 None。"""
        k = self.crash_index
        if k is None:
            return None
        return float(np.linalg.norm(self.vel[k]))

    @property
    def safe_landing(self) -> bool:
        v = self.touchdown_speed
        return v is not None and v < SAFE_TOUCHDOWN_SPEED

    @property
    def horizontal_drift(self) -> float:
        """终点相对目标点的水平偏差，米。"""
        return float(np.linalg.norm(self.pos[-1, :2] - self.target[:2]))

    @property
    def position_rmse(self) -> float:
        """全程三维位置跟踪 RMSE，米。"""
        return float(np.sqrt(np.mean(np.sum((self.pos - self.target) ** 2, axis=1))))

    @property
    def control_effort(self) -> float:
        """控制能量 :math:`\\int \\|\\mathbf u\\|^2 \\, dt`（按中点和算）。"""
        if len(self.t) < 2:
            return 0.0
        u2 = np.sum(self.thrusts ** 2, axis=1)
        return float(np.sum(0.5 * (u2[1:] + u2[:-1]) * np.diff(self.t)))

    @property
    def final_yaw_rate(self) -> float:
        """最后 0.5 s 的平均偏航角速度，deg/s。

        单桨完全失效时四旋翼必须靠机体自转平衡反扭矩，这个量应稳定在非零常数——
        它是"容错控制真的按机制在工作"的直接证据，而不是靠调参凑出来的指标。
        """
        n = max(1, min(len(self.t), int(round(0.5 / (self.t[1] - self.t[0])))))
        return float(np.mean(self.omega[-n:, 2]) * 180.0 / np.pi)

    # ------------------------------------------------------------------ #
    def summary(self) -> Dict[str, object]:
        """一条可直接写进 CSV 的扁平记录。"""
        return {
            "scene": self.scene,
            "scenario": self.scenario,
            "onset": "" if self.onset is None else round(self.onset, 2),
            "crashed": int(self.crashed),
            "min_altitude": round(self.min_altitude, 3),
            "crash_delay": "" if self.crash_delay is None else round(self.crash_delay, 3),
            "touchdown_speed": ("" if self.touchdown_speed is None
                                else round(self.touchdown_speed, 3)),
            "safe_landing": int(self.safe_landing),
            "horiz_drift": round(self.horizontal_drift, 3),
            "pos_rmse": round(self.position_rmse, 3),
            "control_effort": round(self.control_effort, 2),
            "final_yaw_rate": round(self.final_yaw_rate, 1),
            "duration": round(float(self.t[-1]), 2),
        }


def aggregate(results: Sequence[FlightResult]) -> Dict[str, float]:
    """把多组结果汇总成一行对比表。

    Returns
    -------
    dict
        ``survival_rate``   未坠地比例
        ``safe_rate``       安全着陆（触地速度 < 0.5 m/s）比例
        ``mean_min_alt``    平均最低高度，米
        ``mean_crash_delay``故障到坠地的平均时间，秒（仅统计坠地样本）
        ``mean_touchdown``  平均触地速度，m/s（仅统计坠地样本）
        ``mean_pos_rmse``   平均位置 RMSE，米
        ``mean_effort``     平均控制能量
    """
    n = len(results)
    if n == 0:
        raise ValueError("aggregate() 至少需要一条结果")
    crashes = [r for r in results if r.crashed]
    delays = [r.crash_delay for r in crashes if r.crash_delay is not None]
    speeds = [r.touchdown_speed for r in crashes if r.touchdown_speed is not None]
    return {
        "n": float(n),
        "survival_rate": 1.0 - len(crashes) / n,
        "safe_rate": sum(1 for r in results if r.safe_landing) / n,
        "mean_min_alt": float(np.mean([r.min_altitude for r in results])),
        "mean_crash_delay": float(np.mean(delays)) if delays else 0.0,
        "mean_touchdown": float(np.mean(speeds)) if speeds else 0.0,
        "mean_pos_rmse": float(np.mean([r.position_rmse for r in results])),
        "mean_effort": float(np.mean([r.control_effort for r in results])),
    }


def group_summary(results: Sequence[FlightResult], key: str = "scenario") -> Dict[str, Dict]:
    """按 ``key``（``scenario`` 或 ``scene``）分组后再 :func:`aggregate`。"""
    buckets: Dict[str, List[FlightResult]] = {}
    for r in results:
        buckets.setdefault(getattr(r, key), []).append(r)
    return {k: aggregate(v) for k, v in buckets.items()}
