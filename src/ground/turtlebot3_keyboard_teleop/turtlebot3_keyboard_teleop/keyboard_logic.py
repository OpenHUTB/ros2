"""键盘按键到速度指令的纯 Python 映射，不依赖 ROS。"""
import time
from typing import Optional
from dataclasses import dataclass, field


@dataclass
class Limits:
    max_vx: float = 0.5
    max_wz: float = 1.0
    timeout_sec: float = 0.5


_KEY_MAP = {
    "w": (1.0, 0.0),
    "s": (-1.0, 0.0),
    "a": (0.0, 1.0),
    "d": (0.0, -1.0),
    " ": (0.0, 0.0),
}


def key_to_cmd(key, limits=None):
    limits = limits or Limits()
    k = (key or "").lower()
    vx_n, wz_n = _KEY_MAP.get(k, (0.0, 0.0))
    return vx_n * limits.max_vx, wz_n * limits.max_wz


@dataclass
class KeyLatch:
    """键盘锁存：记录最后按键时间，超时自动归零。用 monotonic 防 NTP 跳变。"""
    limits: Limits = field(default_factory=Limits)
    last_time: Optional[float] = None
    cur_vx: float = 0.0
    cur_wz: float = 0.0

    def press(self, key, now=None):
        now = now if now is not None else time.monotonic()
        vx, wz = key_to_cmd(key, self.limits)
        self.last_time = now
        self.cur_vx = vx
        self.cur_wz = wz

    def get_cmd(self, now=None):
        now = now if now is not None else time.monotonic()
        if self.last_time is not None and (now - self.last_time) > self.limits.timeout_sec:
            self.cur_vx = 0.0
            self.cur_wz = 0.0
        return self.cur_vx, self.cur_wz
