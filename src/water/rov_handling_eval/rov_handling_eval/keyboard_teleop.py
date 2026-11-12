# -*- coding: utf-8 -*-
"""键盘 6DOF 遥操作节点。

与常见 teleop 的差别是"按住"语义：终端只在按下与自动重复时吐字符，
**不会发"松开"事件**，所以用「最后活跃时间 + 超时」模拟按住：

    收到字符 → 记录时间戳
    0.35 s 内没再出现 → 认为松开

另外提供 ``H`` 键切换定深保持（发布到 ``/rov/depth_hold``），
用于"有人帮 vs 没人帮"的抗流对比实验。
"""

import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

from .keys import KeyReader, drain_stdin

KEY_AXES = {
    "w": ("surge", +1.0), "s": ("surge", -1.0),
    "a": ("sway", +1.0), "d": ("sway", -1.0),
    "r": ("heave", +1.0), "f": ("heave", -1.0),
    "q": ("yaw", +1.0), "e": ("yaw", -1.0),
    "j": ("roll", +1.0), "l": ("roll", -1.0),
    "i": ("pitch", +1.0), "k": ("pitch", -1.0),
}
AXIS_ORDER = ("surge", "sway", "heave", "roll", "pitch", "yaw")

HELP = """\
──────────────── ROV 手操（6DOF） ────────────────
 纵荡 W/S   横荡 A/D   升沉 R/F
 转向 Q/E   横滚 J/L   俯仰 I/K
 定深保持 H   急停 空格   推力倍率 -/+   退出 X 或 Ctrl-C
──────────────────────────────────────────────────
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("max_surge", 0.6)
        self.declare_parameter("max_sway", 0.5)
        self.declare_parameter("max_heave", 0.4)
        self.declare_parameter("max_yaw", 0.8)
        self.declare_parameter("max_roll", 0.6)
        self.declare_parameter("max_pitch", 0.5)
        self.declare_parameter("hold_timeout", 0.35)
        self.declare_parameter("smoothing_tau", 0.12)
        self.declare_parameter("verbose", True)

        gp = self.get_parameter
        self.limit = {
            "surge": float(gp("max_surge").value), "sway": float(gp("max_sway").value),
            "heave": float(gp("max_heave").value), "yaw": float(gp("max_yaw").value),
            "roll": float(gp("max_roll").value), "pitch": float(gp("max_pitch").value),
        }
        self.rate = float(gp("rate").value)
        self.hold_timeout = float(gp("hold_timeout").value)
        self.tau = max(float(gp("smoothing_tau").value), 1e-3)
        self.verbose = bool(gp("verbose").value)

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_hold = self.create_publisher(Bool, "/rov/depth_hold", 10)

        self.cmd = {a: 0.0 for a in AXIS_ORDER}
        self.last_seen = {}
        self.scale = 1.0
        self.hold = False
        self.quit = False
        self._last_hud = 0.0
        self.tty_ok = sys.stdin.isatty()
        if self.tty_ok:
            drain_stdin()
            print(HELP)
        else:
            self.get_logger().warn("stdin 不是终端：只发布 0，请单独开终端运行本节点")

        self.reader = None
        self.timer = self.create_timer(1.0 / self.rate, self.on_timer)
        self.get_logger().info("keyboard_teleop 启动：%.0f Hz → /cmd_vel" % self.rate)

    # ---------------------------------------------------------------- 主循环
    def on_timer(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 1.0 / self.rate

        if self.tty_ok and self.reader is not None:
            self._consume(now)

        target = {a: 0.0 for a in AXIS_ORDER}
        for key, stamp in list(self.last_seen.items()):
            if now - stamp > self.hold_timeout:
                del self.last_seen[key]
                continue
            axis, sign = KEY_AXES[key]
            target[axis] += sign
        alpha = 1.0 - math.exp(-dt / self.tau)
        for a in AXIS_ORDER:
            target[a] = max(-1.0, min(1.0, target[a])) * self.scale
            self.cmd[a] += (target[a] - self.cmd[a]) * alpha

        msg = Twist()
        msg.linear.x = self.cmd["surge"] * self.limit["surge"]
        msg.linear.y = self.cmd["sway"] * self.limit["sway"]
        msg.linear.z = self.cmd["heave"] * self.limit["heave"]
        msg.angular.x = self.cmd["roll"] * self.limit["roll"]
        msg.angular.y = self.cmd["pitch"] * self.limit["pitch"]
        msg.angular.z = self.cmd["yaw"] * self.limit["yaw"]
        self.pub.publish(msg)

        if self.verbose and now - self._last_hud > 0.25:
            self._last_hud = now
            self._hud()

        if self.quit:
            self.timer.cancel()
            rclpy.shutdown()

    def _consume(self, now):
        for ch in self.reader.poll(0.0):
            low = ch.lower()
            if ch == " ":
                self.last_seen.clear()
                self.cmd = {a: 0.0 for a in AXIS_ORDER}
            elif low == "h":
                self.hold = not self.hold
                m = Bool()
                m.data = self.hold
                self.pub_hold.publish(m)
            elif low == "x":
                self.quit = True
                return
            elif ch in ("+", "="):
                self.scale = min(1.5, self.scale + 0.1)
            elif ch == "-":
                self.scale = max(0.2, self.scale - 0.1)
            elif low in KEY_AXES:
                self.last_seen[low] = now

    def _hud(self):
        bars = []
        for a in AXIS_ORDER:
            v = self.cmd[a]
            n = int(round(abs(v) * 6))
            bar = ("█" * n).ljust(6) if v >= 0 else ("█" * n).rjust(6)
            bars.append("%s%+.2f|%s|" % (a[0].upper(), v, bar))
        sys.stdout.write("\r倍率%.1f 定深%s  %s" % (self.scale, "开" if self.hold else "关", " ".join(bars)))
        sys.stdout.flush()

    # ---------------------------------------------------------------- 生命周期
    def run(self):
        if not self.tty_ok:
            try:
                rclpy.spin(self)
            except (KeyboardInterrupt, SystemExit):
                pass
            return
        with KeyReader() as reader:
            self.reader = reader
            try:
                rclpy.spin(self)
            except (KeyboardInterrupt, SystemExit):
                pass


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    finally:
        print()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
