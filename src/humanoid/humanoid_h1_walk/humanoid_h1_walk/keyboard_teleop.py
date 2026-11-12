"""键盘遥控节点：方向键发布 ``/cmd_vel``，把「上/下/左/右」变成速度指令。

速度指令是策略的输入之一，范围为 vx ∈ [-0.5, 1.0] m/s、wz ∈ [-1.0, 1.0] rad/s，
超出后策略跟踪不上、容易失稳，所以在这里先夹紧。

    ros2 run humanoid_h1_walk keyboard_teleop
    ros2 run humanoid_h1_walk keyboard_teleop --ros-args -p step:=0.1

Linux/macOS 用 termios 读单键，Windows 用 msvcrt；两者都是非阻塞读。
"""

from __future__ import annotations

import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# 方向键在终端里发的是 ESC [ A/B/C/D
ARROWS = {"\x1b[A": (0, +1), "\x1b[B": (0, -1), "\x1b[C": (2, +1), "\x1b[D": (2, -1)}
# WASD 作为备用（部分终端不上报方向键）
WASD = {"w": (0, +1), "s": (0, -1), "d": (2, +1), "a": (2, -1)}


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("humanoid_h1_walk_teleop")
        self.declare_parameter("step", 0.1)
        self.declare_parameter("turn_step", 0.2)
        self.declare_parameter("rate", 20.0)

        self.step = float(self.get_parameter("step").value)
        self.turn_step = float(self.get_parameter("turn_step").value)
        self.cmd = [0.0, 0.0]
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_timer(1.0 / float(self.get_parameter("rate").value), self._tick)
        self.get_logger().info("方向键 ↑↓ 调速、←→ 转向，空格停止，q 退出")

    def _read_key(self) -> str | None:
        if sys.platform == "win32":
            import msvcrt

            if not msvcrt.kbhit():
                return None
            ch = msvcrt.getwch()
            if ch in ("\x00", "\xe0"):  # Windows 方向键前缀
                return {"H": "\x1b[A", "P": "\x1b[B", "M": "\x1b[C", "K": "\x1b[D"}.get(
                    msvcrt.getwch()
                )
            return ch
        import select

        if not select.select([sys.stdin], [], [], 0)[0]:
            return None
        return sys.stdin.read(1)

    def _apply(self, key: str) -> bool:
        if key in ("q", "Q", "\x03"):
            return False
        if key == " ":
            self.cmd = [0.0, 0.0]
        else:
            change = ARROWS.get(key) or WASD.get(key.lower())
            if change is None:
                return True
            index, sign = change
            delta = self.step if index == 0 else self.turn_step
            self.cmd[index] = self.cmd[index] + sign * delta
        self.cmd[0] = min(1.0, max(-0.5, self.cmd[0]))
        self.cmd[1] = min(1.0, max(-1.0, self.cmd[1]))
        return True

    def _tick(self) -> None:
        msg = Twist()
        msg.linear.x = self.cmd[0]
        msg.angular.z = self.cmd[1]
        self.pub.publish(msg)


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = KeyboardTeleop()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            key = node._read_key()
            if key is not None and not node._apply(key):
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
