"""安全辅助：传感器数据新鲜度跟踪与参数合法性校验。

本模块刻意不依赖 rclpy，只处理纯数值/字典逻辑，因此可以在无 ROS 环境的
单元测试里直接覆盖（ROS2 节点本身必须安装 rclpy 才能导入）。

两类问题由它兜底：

1. 话题断流：节点若继续沿用最后一帧 /odom 或 /scan 解算速度，等于在
   过期状态上闭环，机器人会照着陈旧位姿一直跑。这里记录每个数据源的
   最后到达时刻，超时后调用方必须停车（odom）或停止避障（scan）。
2. 参数非法：target_speed 为 0、负数或 NaN 会让控制器输出恒零或 NaN，
   与其在运行中产生难以定位的异常，不如在启动时就拒绝并回退默认值。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class FreshnessTracker:
    """跟踪若干数据源的最后到达时刻，判定其是否已过期。

    时间统一用**秒**（float）。ROS2 侧可用
    ``node.get_clock().now().nanoseconds / 1e9`` 取得，测试里直接传数字。
    """

    timeouts: dict[str, float] = field(default_factory=dict)
    last_seen: dict[str, float | None] = field(default_factory=dict)

    def register(self, source: str, timeout: float) -> None:
        """登记一个数据源及其超时阈值（秒）。"""
        self.timeouts[source] = float(timeout)
        self.last_seen.setdefault(source, None)

    def mark(self, source: str, now: float) -> None:
        """记录该数据源在 now 时刻收到一帧新数据。"""
        self.last_seen[source] = float(now)

    def age(self, source: str, now: float) -> float:
        """距上一帧数据过去了多久；从未收到过返回 inf。"""
        last = self.last_seen.get(source)
        if last is None:
            return math.inf
        return max(float(now) - last, 0.0)

    def is_stale(self, source: str, now: float) -> bool:
        """该数据源是否已超时（含从未收到过数据的情况）。"""
        return self.age(source, now) > self.timeouts.get(source, math.inf)

    def is_fresh(self, source: str, now: float) -> bool:
        return not self.is_stale(source, now)


def sanitize_target_speed(
    value: float,
    default: float = 0.8,
    vmin: float = 0.05,
    vmax: float = 3.0,
) -> tuple[float, str | None]:
    """校验期望速度，返回 (可用速度, 警告信息或 None)。

    - 非有限值（NaN/inf）或小于等于 vmin：视为非法，回退 default
    - 超过 vmax：截断到 vmax（仍给出警告，但不拒绝运行）
    - 合法值：原样返回，无警告
    """
    try:
        v = float(value)
    except (TypeError, ValueError):
        return float(default), f"target_speed 无法解析为数值: {value!r}，已回退 {default}"

    if not math.isfinite(v):
        return float(default), f"target_speed 非法 ({value!r})，已回退 {default}"
    if v <= vmin:
        return float(default), f"target_speed={v} 过小（应 > {vmin}），已回退 {default}"
    if v > vmax:
        return float(vmax), f"target_speed={v} 超过上限 {vmax}，已截断"
    return v, None
