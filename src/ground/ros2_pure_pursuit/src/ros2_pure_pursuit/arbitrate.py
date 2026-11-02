"""速度仲裁：决定一个控制周期内最终下发哪一组 (v, w)。

仲裁优先级（从高到低）：

    1. 到达终点   -> 强制停车 (0, 0)，后续全部跳过
    2. 纯追踪     -> 路径跟踪解算出的 (v, w)
    3. 避障覆盖   -> 在纯追踪输出上叠加转向偏置与减速系数

之所以单独成模块且刻意不依赖 rclpy，是为了让这段最容易出错的
优先级判断可以被单元测试直接覆盖（ROS2 节点本身需要 ROS 环境才能导入）。

历史缺陷：早期实现把终点判断写成

    if 避障开启 and 有激光:
        ...
    elif info.finished:
        v = 0.0

一旦开着避障且收到激光数据，`elif` 分支永远不会执行，机器人到达终点后
不会停车，会继续绕着终点跑。该缺陷由 test_arbitration.py 中的
test_finished_wins_over_avoidance 回归锁定。
"""

from __future__ import annotations

from dataclasses import dataclass

from .obstacle import AvoidInfo


@dataclass
class ArbitrationResult:
    """一次仲裁的结果。"""

    v: float
    w: float
    reason: str          # "finished" | "pursuit" | "avoidance"
    applied_avoidance: bool


def arbitrate(
    v_pursuit: float,
    w_pursuit: float,
    finished: bool,
    avoid: AvoidInfo | None = None,
) -> ArbitrationResult:
    """按优先级裁决本周期的速度指令。

    Args:
        v_pursuit: 纯追踪解算出的线速度。
        w_pursuit: 纯追踪解算出的角速度。
        finished: 是否已到达路径终点。
        avoid: 避障建议；为 None 表示未开启避障或本周期无激光数据。

    Returns:
        ArbitrationResult，其中 v/w 即最终下发值。
    """
    v, w, reason = 0.0, 0.0, "pursuit"

    # 1. 终点最优先：直接停车，不再做任何后续计算
    if finished:
        return ArbitrationResult(0.0, 0.0, "finished", False)

    # 2. 未到终点：采纳纯追踪输出
    v, w = float(v_pursuit), float(w_pursuit)

    # 3. 避障作为安全兜底覆盖层
    applied = False
    if avoid is not None and (avoid.turn_bias != 0.0 or avoid.slow_factor != 1.0):
        w += avoid.turn_bias
        v *= avoid.slow_factor
        applied = True
        reason = "avoidance"

    return ArbitrationResult(v, w, reason, applied)
