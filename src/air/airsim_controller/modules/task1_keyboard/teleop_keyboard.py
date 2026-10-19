"""键盘遥控节点：把按键实时映射为世界系速度指令。

按键映射（与 AirSim 官方键盘例程一致）：
    W / S   前后（vx）
    A / D   左右（vy）
    R / F   上升 / 下降（vz，z 向上为正）
    Q / E   偏航 左 / 右（角速度）
    T       起飞，  Space 悬停，  L 降落，  Esc 退出

依赖：``pip install pynput``。无 GUI 的服务器上本模块无法采集键盘，属预期行为。
"""

import threading
import time

import numpy as np

try:
    from pynput import keyboard as kb
    _HAS_PYNPUT = True
except ImportError:  # pragma: no cover
    kb = None
    _HAS_PYNPUT = False


# 按键 -> 三轴速度增量（m/s）与偏航角速度（deg/s）
_KEY_BINDINGS = {
    'w': np.array([1.0, 0.0, 0.0]),
    's': np.array([-1.0, 0.0, 0.0]),
    'd': np.array([0.0, 1.0, 0.0]),
    'a': np.array([0.0, -1.0, 0.0]),
    'r': np.array([0.0, 0.0, 1.0]),
    'f': np.array([0.0, 0.0, -1.0]),
}


class KeyboardTeleop:
    def __init__(self, client, speed=1.0, yaw_rate=30.0, rate_hz=10.0):
        self.client = client
        self.speed = speed
        self.yaw_rate = yaw_rate
        self.dt = 1.0 / rate_hz
        self._pressed = set()
        self._yaw_cmd = 0.0
        self._running = True
        self._listener = None

    # ------------------------------------------------------------ 按键采集
    def _on_press(self, key):
        try:
            ch = key.char.lower()
        except AttributeError:
            ch = None
        if ch in _KEY_BINDINGS:
            self._pressed.add(ch)
        elif ch == 'q':
            self._yaw_cmd = self.yaw_rate
        elif ch == 'e':
            self._yaw_cmd = -self.yaw_rate
        elif ch == ' ':
            self._pressed.clear()
        elif ch == 'l':
            self.client.land()
        elif ch == 't':
            self.client.start()
        elif key == kb.Key.esc:
            self._running = False
            return False

    def _on_release(self, key):
        try:
            ch = key.char.lower()
        except AttributeError:
            return
        if ch in _KEY_BINDINGS and ch in self._pressed:
            self._pressed.remove(ch)
        if ch in ('q', 'e'):
            self._yaw_cmd = 0.0

    # ------------------------------------------------------------ 主循环
    def _target_velocity(self):
        v = np.zeros(3)
        for ch in self._pressed:
            v += _KEY_BINDINGS[ch]
        norm = np.linalg.norm(v)
        if norm > 1e-6:
            v = v / max(norm, 1.0) * self.speed
        return v

    def run(self):
        if not _HAS_PYNPUT:
            raise RuntimeError("缺少 pynput，请先 pip install pynput")
        self.client.start()
        print(self.__doc__)
        with kb.Listener(on_press=self._on_press, on_release=self._on_release) as listener:
            self._listener = listener
            while self._running:
                v = self._target_velocity()
                self.client.move_by_velocity(v[0], v[1], v[2],
                                             yaw_rate=np.radians(self._yaw_cmd),
                                             duration=self.dt)
                time.sleep(self.dt)
        self.client.destroy()
