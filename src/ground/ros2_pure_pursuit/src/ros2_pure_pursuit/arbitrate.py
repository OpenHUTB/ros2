"""速度仲裁：决定一个控制周期内最终下发哪一组 (v, w)。

仲裁优先级（从高到低）：

    0. 定位数据过期 -> 强制停车 (0, 0)，位姿不可信时不能闭环
    1. 到达终点     -> 强制停车 (0, 0)，后续全部跳过
    2. 纯追踪       -> 路径跟踪解算出的 (v, w)
    3. 避障覆盖     -> 在纯追踪输出上叠加转向偏置与减速系数
    4. 危险距离内   -> 急停 v=0，但保留转向以便原地转向脱离

最后统一把角速度截断到 ±max_angular，保证下发的指令在车辆可执行范围内。

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
    #: "stale_odom" | "finished" | "emergency_stop" | "avoidance" | "pursuit"
    reason: str
    applied_avoidance: bool


def arbitrate(
    v_pursuit: float,
    w_pursuit: float,
    finished: bool,
    avoid: AvoidInfo | None = None,
    odom_stale: bool = False,
    max_angular: float = 1.8,
) -> ArbitrationResult:
    """按优先级裁决本周期的速度指令。

    Args:
        v_pursuit: 纯追踪解算出的线速度。
        w_pursuit: 纯追踪解算出的角速度。
        finished: 是否已到达路径终点。
        avoid: 避障建议；为 None 表示未开启避障、无激光数据或数据已过期。
        odom_stale: 定位数据是否已超时（含从未收到过）。为 True 时强制停车。
        max_angular: 角速度幅值上限 (rad/s)，防止转向偏置叠加后超出可执行范围。

    Returns:
        ArbitrationResult，其中 v/w 即最终下发值。
    """
    # 0. 定位过期：状态不可信，直接停车（优先级高于终点判断，
    #    因为此时连“是否到终点”都无法确认）
    if odom_stale:
        return ArbitrationResult(0.0, 0.0, "stale_odom", False)

    # 1. 终点：直接停车，不再做任何后续计算
    if finished:
        return ArbitrationResult(0.0, 0.0, "finished", False)

    # 2. 未到终点：采纳纯追踪输出
    v, w = float(v_pursuit), float(w_pursuit)
    reason = "pursuit"
    applied = False

    # 3. 避障作为安全兜底覆盖层
    if avoid is not None and (avoid.turn_bias != 0.0 or avoid.slow_factor != 1.0):
        w += avoid.turn_bias
        v *= avoid.slow_factor
        applied = True
        reason = "avoidance"

        # 4. 已进入危险距离：减速不够，必须停住；保留转向以便原地转离
        if avoid.emergency:
            v = 0.0
            reason = "emergency_stop"

    # 5. 角速度统一限幅，避免叠加偏置后超出车辆能力
    w = float(max(-max_angular, min(max_angular, w)))

    return ArbitrationResult(v, w, reason, applied)
