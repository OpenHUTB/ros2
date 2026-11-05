"""键盘遥控节点：控制 G1 的上半身姿态，并发布速度指令。

上半身姿态由 PD 跟踪，腿部的平衡由神经网络 + 平衡基元负责。因此
「按键摆臂/弯腰 → 质心偏移 → 平衡器自动补偿」这个过程是可见且物理真实的。

按键
----
腰部
    W / S   前倾 / 后仰（waist_pitch）
    A / D   腰部左转 / 右转（waist_yaw）
    Q / E   左倾 / 右倾（waist_roll）
左臂
    1 / 2   肩前抬 / 放下（shoulder_pitch）
    3 / 4   肩外展 / 内收（shoulder_roll）
    5 / 6   屈肘 / 伸肘（elbow）
右臂
    7 / 8   肩前抬 / 放下
    9 / 0   肩外展 / 内收
    - / =   屈肘 / 伸肘
其它
    R       上半身全部复位
    I / K   速度指令 vx ±0.1 m/s（范围 -0.3 ~ +0.5）
    J / L   速度指令 wz ±0.1 rad/s
    空格     速度指令清零
    X       退出

只依赖标准库 + rclpy，Linux 终端下用 termios 读取无回显按键。
"""

from __future__ import annotations

import select
import sys
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .sim_core import UPPER_JOINTS

try:  # 仅 Linux 可用；Windows 上退化为逐行输入
    import termios
    import tty
except ImportError:  # pragma: no cover
    termios = None
    tty = None

HELP = __doc__

# 关节名 -> 在 UPPER_JOINTS 中的下标
_IDX = {n: i for i, n in enumerate(UPPER_JOINTS)}

# 按键 -> (关节下标, 增量, 上下限)
# 上下限由实测标定（见 README 5.3 / 6.2）：肩前抬超过 1.2 rad 会破坏平衡，
# 所以压到 1.00；腰部 pitch/roll 的关节限位本身就是 0.52，指令再大也只是顶住限位。
KEYMAP: dict[str, tuple[int, float, float]] = {
    "w": (_IDX["waist_pitch"], +0.08, 0.50),
    "s": (_IDX["waist_pitch"], -0.08, 0.50),
    "a": (_IDX["waist_yaw"], +0.15, 0.60),
    "d": (_IDX["waist_yaw"], -0.15, 0.60),
    "q": (_IDX["waist_roll"], +0.08, 0.40),
    "e": (_IDX["waist_roll"], -0.08, 0.40),
    "1": (_IDX["left_shoulder_pitch"], +0.20, 1.00),
    "2": (_IDX["left_shoulder_pitch"], -0.20, 1.00),
    "3": (_IDX["left_shoulder_roll"], +0.15, 1.40),
    "4": (_IDX["left_shoulder_roll"], -0.15, 1.40),
    "5": (_IDX["left_elbow"], +0.25, 1.80),
    "6": (_IDX["left_elbow"], -0.25, 1.80),
    "7": (_IDX["right_shoulder_pitch"], +0.20, 1.00),
    "8": (_IDX["right_shoulder_pitch"], -0.20, 1.00),
    "9": (_IDX["right_shoulder_roll"], -0.15, 1.40),
    "0": (_IDX["right_shoulder_roll"], +0.15, 1.40),
    "-": (_IDX["right_elbow"], +0.25, 1.80),
    "=": (_IDX["right_elbow"], -0.25, 1.80),
}


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("humanoid_keyboard_teleop")
        # 速度指令范围与训练时一致（策略见过的分布）
        self.declare_parameter("max_linear", 0.5)
        self.declare_parameter("min_linear", -0.3)
        self.declare_parameter("max_angular", 0.4)
        self.declare_parameter("linear_step", 0.1)
        self.declare_parameter("angular_step", 0.1)

        self.max_linear = float(self.get_parameter("max_linear").value)
        self.min_linear = float(self.get_parameter("min_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)
        self.lin_step = float(self.get_parameter("linear_step").value)
        self.ang_step = float(self.get_parameter("angular_step").value)

        self.vx = 0.0
        self.wz = 0.0
        self.upper = np.zeros(len(UPPER_JOINTS))
        self.running = True

        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_upper = self.create_publisher(Float64MultiArray, "humanoid/upper_body", 10)
        self.create_timer(0.05, self._publish)
        self.get_logger().info(
            "键盘遥控已启动：WASD/QE 控制腰部，1-6 左臂，7-0/-= 右臂，R 复位，X 退出"
        )

    def _publish(self) -> None:
        cmd = Twist()
        cmd.linear.x = self.vx
        cmd.angular.z = self.wz
        self.pub_cmd.publish(cmd)

        msg = Float64MultiArray()
        msg.data = self.upper.astype(float).tolist()
        self.pub_upper.publish(msg)

    def handle_key(self, key: str) -> None:
        if key in KEYMAP:
            idx, delta, limit = KEYMAP[key]
            self.upper[idx] = float(np.clip(self.upper[idx] + delta, -limit, limit))
            self.get_logger().info(
                f"{UPPER_JOINTS[idx]} = {self.upper[idx]:+.2f} rad"
            )
        elif key == "r":
            self.upper[:] = 0.0
            self.get_logger().info("上半身已复位")
        elif key == "i":
            self.vx = min(self.vx + self.lin_step, self.max_linear)
            self.get_logger().info(f"cmd_vel: vx={self.vx:+.2f} m/s")
        elif key == "k":
            self.vx = max(self.vx - self.lin_step, self.min_linear)
            self.get_logger().info(f"cmd_vel: vx={self.vx:+.2f} m/s")
        elif key == "j":
            self.wz = min(self.wz + self.ang_step, self.max_angular)
            self.get_logger().info(f"cmd_vel: wz={self.wz:+.2f} rad/s")
        elif key == "l":
            self.wz = max(self.wz - self.ang_step, -self.max_angular)
            self.get_logger().info(f"cmd_vel: wz={self.wz:+.2f} rad/s")
        elif key == " ":
            self.vx = self.wz = 0.0
            self.get_logger().info("cmd_vel 已清零")
        elif key == "x":
            self.running = False


def _read_key_blocking() -> str:
    """读一个按键；Linux 下用 cbreak 模式，避免回车。"""
    if termios is None:
        return sys.stdin.readline().strip()[:1].lower()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if r:
                return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main(argv=None):
    rclpy.init(args=argv)
    node = KeyboardTeleop()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(HELP)
    try:
        while node.running:
            key = _read_key_blocking()
            if key:
                node.handle_key(key.lower())
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.vx = node.wz = 0.0
        node._publish()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
