"""节点每个控制周期的决策：里程计可用性判定 + 检测 + 脱困，全部不依赖 ROS。

单独抽出来的原因是**启动期的边界条件必须能离线测**：
``/odom`` 还没来、任务层就已经在发速度指令时，节点必须输出零速（fail closed），
而不是把速度透传到底盘。把这些判断留在 ``node.py`` 里就只能靠实车上试，
所以这里提供一个纯函数入口 ``evaluate()``，节点只做话题收发与消息打包。
"""
from __future__ import annotations

import math

from .detect import CAUSE_NONE, Sample, StuckDetector, Verdict
from .recovery import RecoverySupervisor
from .robot import Twist


class OdomWatch:
    """跟踪里程计是否可用。

    关键点：**"从未收到"与"刚收到"必须区分开**。若用一个时间戳初值 0 代表
    "还没收到"，那么 ``now - 0`` 得到的是"很久没更新"还是"刚好更新"取决于
    时钟起点，早期实现取 ``max(..., 0)`` 的写法会让启动期伪装成"新鲜数据"，
    于是任务层的速度指令会被当成正常情况透传。
    """

    def __init__(self) -> None:
        self._last_t = 0.0
        self._seen = False
        self.count = 0

    @property
    def seen(self) -> bool:
        return self._seen

    def update(self, now: float) -> None:
        self._last_t = now
        self._seen = True
        self.count += 1

    def age(self, now: float) -> float:
        """数据年龄 [s]；从未收到时返回 ``inf``（表示"不可用"，而非"很新"）。"""
        if not self._seen:
            return math.inf
        return max(now - self._last_t, 0.0)


def evaluate(
    now: float,
    task: Twist,
    watch: OdomWatch,
    detector: StuckDetector,
    supervisor: RecoverySupervisor,
    *,
    v_odom: float,
    w_odom: float,
    w_imu: float | None,
    range_front: float | None,
    pose: tuple[float, float, float],
):
    """跑一个周期，返回 ``(Command, Verdict, odom_age)``。

    里程计不可用（从未收到，或超过 ``stale_odom`` 未更新）时：

    * 检测器缓冲清空——不可用期间"有指令却零位移"是假证据，留着会在里程计
      恢复的瞬间被误判成卡死，进而触发一次无谓的倒车脱困；
    * 结论强制为"无故障"，让 ``supervisor`` 走自己的 stale 分支输出零速。
    """
    age = watch.age(now)
    stale = supervisor.cfg.stale_odom

    if age > stale:
        detector.reset()
        verdict = Verdict()
    else:
        verdict = detector.update(
            Sample(
                t=now,
                v_cmd=task.v,
                w_cmd=task.w,
                v_odom=v_odom,
                w_odom=w_odom,
                w_imu=w_imu,
                range_front=range_front,
            )
        )

    imu_gap = abs(w_odom - w_imu) if w_imu is not None else 0.0
    out = supervisor.step(now, task, verdict, age, pose, imu_gap)
    return out, verdict, age


def odom_available(age: float, stale_odom: float) -> bool:
    """里程计是否可用于判断卡死（供状态上报复用）。"""
    return math.isfinite(age) and age <= stale_odom


def verdict_is_clean(verdict: Verdict) -> bool:
    """便于测试与日志：是否没有任何故障结论。"""
    return verdict.cause == CAUSE_NONE
