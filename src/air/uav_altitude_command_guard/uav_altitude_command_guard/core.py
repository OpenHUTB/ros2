"""独立于 ROS 的高度目标约束."""
import math


class AltitudeGuard:
    """按时间步约束目标变化，在指令超时后保持当前输出."""

    def __init__(self, minimum=0.4, maximum=3.0, rate=0.5, timeout=0.6, initial=1.5):
        values = (minimum, maximum, rate, timeout, initial)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('高度与时间参数必须为有限数值')
        if minimum >= maximum or rate <= 0 or timeout <= 0 or not minimum <= initial <= maximum:
            raise ValueError('高度范围、变化率、超时时间或初值不合法')
        self.minimum, self.maximum, self.rate, self.timeout = minimum, maximum, rate, timeout
        self.value = self.target = float(initial)
        self.last_command = None
        self.last_tick = None

    def command(self, height, now):
        """接收新目标；拒绝非法输入，并且不刷新有效指令时间."""
        if not math.isfinite(now) or (self.last_tick is not None and now < self.last_tick):
            raise ValueError('指令时间必须有限且不能早于上次执行时间')
        if self.last_command is not None and now < self.last_command:
            raise ValueError('指令时间不能倒退')
        if not math.isfinite(height):
            return False
        self.target = min(self.maximum, max(self.minimum, height))
        self.last_command = now
        return True

    def tick(self, now):
        """返回安全目标和状态；严格使用相邻执行时刻的实际间隔."""
        if not math.isfinite(now) or (self.last_tick is not None and now < self.last_tick):
            raise ValueError('执行时间必须有限且不能倒退')
        if self.last_command is not None and now < self.last_command:
            raise ValueError('执行时间不能早于已接收指令')
        dt = 0.0 if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        stale = self.last_command is None or now - self.last_command >= self.timeout
        if stale:
            # 清除尚未完成的旧目标，避免下一次有效指令前继续爬升。
            self.target = self.value
            return self.value, '超时保持'
        step = self.rate * dt
        self.value += max(-step, min(step, self.target - self.value))
        return self.value, '正常跟随'
