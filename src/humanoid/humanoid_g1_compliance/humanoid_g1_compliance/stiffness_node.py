"""刚度调节节点：终端键盘改 5 个刚度通道。

按键（与上游 stiffness_control 一致）：

    1 2 3 4 5   提高对应通道 ×1.25（长按连发）
    q w e r t   降低对应通道 ÷1.25
    0           复位到 [250, 250, 250, 55, 55]

通道顺序：[K_lin_la, K_lin_ra, K_lin_to, K_ang_la, K_ang_ra]
线性刚度带 [10, 500] N/m，角刚度带 [10, 100] N·m/rad（训练分布）。
"""

from __future__ import annotations

import sys
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from .policy import DEFAULT_STIFFNESS_5, STIFF_BAND, STIFF_NAMES

UP_KEYS = '12345'
DOWN_KEYS = 'qwert'
STEP = 1.25


class StiffnessNode(Node):
    def __init__(self):
        super().__init__('g1_compliance_stiffness')
        self.declare_parameter('stiffness', list(DEFAULT_STIFFNESS_5))
        self.vals = np.asarray(self.get_parameter('stiffness').value, dtype=np.float64)
        self._lock = threading.Lock()

        self.pub = self.create_publisher(Float32MultiArray, '/humanoid/stiffness_cmd', 10)
        self.create_timer(0.2, self._republish)
        self._print_help()

        threading.Thread(target=self._read_stdin, daemon=True).start()

    def _print_help(self):
        print('刚度通道：')
        for i, n in enumerate(STIFF_NAMES):
            band = STIFF_BAND['K_ang' if 'ang' in n else 'K_lin']
            print(f'  {i + 1} / {DOWN_KEYS[i]}  {n:10s} = {self.vals[i]:7.1f}'
                  f'   范围 [{band[0]:.0f}, {band[1]:.0f}]')
        print('  0 = 复位     Ctrl+C 退出\n')

    def _read_stdin(self):
        try:
            for line in sys.stdin:
                for ch in line.strip():
                    self._apply(ch)
        except Exception:  # noqa: BLE001  终端关闭
            pass

    def _apply(self, ch: str):
        with self._lock:
            if ch == '0':
                self.vals = np.asarray(DEFAULT_STIFFNESS_5, dtype=np.float64)
            elif ch in UP_KEYS:
                i = UP_KEYS.index(ch)
                self.vals[i] = self._clip(i, self.vals[i] * STEP)
            elif ch in DOWN_KEYS:
                i = DOWN_KEYS.index(ch)
                self.vals[i] = self._clip(i, self.vals[i] / STEP)
            else:
                return
            print('  ' + '  '.join(f'{n}={v:.0f}' for n, v in zip(STIFF_NAMES, self.vals)))
        self._publish()

    @staticmethod
    def _clip(i: int, v: float) -> float:
        lo, hi = STIFF_BAND['K_ang' if 'ang' in STIFF_NAMES[i] else 'K_lin']
        return float(np.clip(v, lo, hi))

    def _publish(self):
        msg = Float32MultiArray()
        msg.data = [float(v) for v in self.vals]
        self.pub.publish(msg)

    def _republish(self):
        self._publish()


def main(args=None):
    rclpy.init(args=args)
    node = StiffnessNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
