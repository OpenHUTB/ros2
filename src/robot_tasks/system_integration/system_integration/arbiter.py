"""速度指令仲裁：模式切换、看门狗、急停、限幅。

这一层不依赖 ROS，方便单测；ROS 节点只做订阅/发布的外壳。
"""

from __future__ import annotations

import math
import time

MODE_MANUAL = 'manual'
MODE_AUTO = 'auto'
_MODE_ALIAS = {'manual': MODE_MANUAL, '手动': MODE_MANUAL,
               'auto': MODE_AUTO, '自动': MODE_AUTO}


def _clamp(value, low, high):
    return low if value < low else (high if value > high else value)


class VelocityArbiter:
    """把手动与自动两路速度指令合成一路，并负责安全兜底。

    输入：
        feed_manual(v, w) / feed_auto(v, w)  两路各自带时间戳的最近指令
    输出：
        step() -> (v, w, status)  选中的速度与状态说明
    """

    def __init__(self, timeout: float = 0.5, max_linear: float = 1.0,
                 max_angular: float = 1.5):
        self.timeout = max(0.05, float(timeout))
        self.max_linear = abs(float(max_linear))
        self.max_angular = abs(float(max_angular))

        self.mode = MODE_MANUAL
        self.estop = False
        self._data = {
            MODE_MANUAL: {'v': 0.0, 'w': 0.0, 't': -1e9, 'seq': 0},
            MODE_AUTO: {'v': 0.0, 'w': 0.0, 't': -1e9, 'seq': 0},
        }
        self.mode_switches = 0
        self.estop_events = 0

    # ---------- 输入 ----------
    def set_mode(self, mode):
        """mode 支持 manual/auto（也接受 手动/自动），非法值返回 False。"""
        key = _MODE_ALIAS.get(str(mode).strip().lower())
        if key is None:
            return False
        if key != self.mode:
            self.mode = key
            self.mode_switches += 1
        return True

    def set_estop(self, on, now=None):
        on = bool(on)
        if on and not self.estop:
            self.estop_events += 1
        self.estop = on

    def feed(self, source, v, w, now=None):
        key = _MODE_ALIAS.get(str(source).strip().lower())
        if key is None:
            return False
        v, w = float(v), float(w)
        # 非有限值一律拒收：NaN 参与比较恒为 False，会绕过限幅直接下发到 /cmd_vel
        if not (math.isfinite(v) and math.isfinite(w)):
            return False
        now = time.monotonic() if now is None else float(now)
        if not math.isfinite(now):
            return False
        self._data[key] = {'v': v, 'w': w,
                           't': now, 'seq': self._data[key]['seq'] + 1}
        return True

    def feed_manual(self, v, w, now=None):
        return self.feed(MODE_MANUAL, v, w, now)

    def feed_auto(self, v, w, now=None):
        return self.feed(MODE_AUTO, v, w, now)

    # ---------- 输出 ----------
    def is_stale(self, source=None, now=None):
        now = time.monotonic() if now is None else float(now)
        d = self._data[source or self.mode]
        return d['seq'] == 0 or (now - d['t']) > self.timeout

    def step(self, now=None):
        """返回 (v, w, status)，status 含 mode/estop/source/reason/age。"""
        now = time.monotonic() if now is None else float(now)

        status = {'mode': self.mode, 'estop': self.estop,
                  'source': 'none', 'reason': 'ok', 'age': 0.0}

        if self.estop:
            status.update(source='estop', reason='estop')
            return 0.0, 0.0, status

        d = self._data[self.mode]
        if d['seq'] == 0:
            status.update(source='none', reason='no_data')
            return 0.0, 0.0, status

        age = now - d['t']
        status['age'] = age
        if age > self.timeout:
            status.update(source=self.mode, reason='stale')
            return 0.0, 0.0, status

        v = _clamp(d['v'], -self.max_linear, self.max_linear)
        w = _clamp(d['w'], -self.max_angular, self.max_angular)
        status['source'] = self.mode
        return v, w, status
