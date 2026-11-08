#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""数字指令控制节点：防抖识别结果 -> 动作映射 -> turtlesim 运动控制。

数字与动作映射（小键盘直觉布局）：
    2 前进 | 8 后退 | 4 左移 | 6 右移      （体坐标平移，开环定时长）
    1 左转45° | 3 右转45° | 7 左转90° | 9 右转90° | 0 原地转一圈
                                           （闭环 P 控制器，基于 /turtle1/pose）
    5 停止

话题：
    订阅：digit_topic    std_msgs/Int32     检测节点发布的数字（-1 无目标）
          conf_topic     std_msgs/Float32   置信度
          /turtle1/pose  turtlesim_msgs/Pose 乌龟位姿（旋转闭环）
    发布：/turtle1/cmd_vel            geometry_msgs/Twist  速度指令
          /mnist_digit_control/action std_msgs/String      当前执行的动作名

参数：stable_frames、conf_threshold、cooldown、linear_speed、action_duration、
      angular_kp、angular_max、angular_tol、turn_deg_* 等，见 declare_parameter。
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String

try:
    from turtlesim_msgs.msg import Pose
except ImportError:  # 兼容消息绑定在 turtlesim.msg 下的环境
    from turtlesim.msg import Pose

# 动作表：(中文名, ASCII 名, 类型, 参数)。ASCII 名经 action 话题供 detector
# 在 OpenCV 画面上叠加显示（cv2.putText 不支持中文），日志仍使用中文名。
ACTIONS = {
    2: ("前进", "forward", "translate", {"vx": 1.0}),
    8: ("后退", "backward", "translate", {"vx": -1.0}),
    4: ("左移", "left", "translate", {"vy": 1.0}),
    6: ("右移", "right", "translate", {"vy": -1.0}),
    1: ("左转45度", "turn L 45", "rotate", {"deg": 45.0}),
    3: ("右转45度", "turn R 45", "rotate", {"deg": -45.0}),
    7: ("左转90度", "turn L 90", "rotate", {"deg": 90.0}),
    9: ("右转90度", "turn R 90", "rotate", {"deg": -90.0}),
    0: ("原地旋转360度", "rotate 360", "rotate", {"deg": 360.0}),
    5: ("停止", "stop", "stop", {}),
}


def wrap_angle(a: float) -> float:
    """把角度归一化到 (-pi, pi]。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class DigitCommandNode(Node):
    def __init__(self) -> None:
        super().__init__("digit_command")
        self.declare_parameter("digit_topic", "/digit_detector/digit")
        self.declare_parameter("conf_topic", "/digit_detector/confidence")
        self.declare_parameter("stable_frames", 6)
        self.declare_parameter("conf_threshold", 0.7)
        self.declare_parameter("cooldown", 0.8)
        self.declare_parameter("linear_speed", 1.0)
        self.declare_parameter("action_duration", 1.0)
        self.declare_parameter("angular_kp", 3.0)
        self.declare_parameter("angular_max", 2.0)
        self.declare_parameter("angular_tol", 0.03)
        self.declare_parameter("control_hz", 20.0)

        gp = self.get_parameter
        self.stable_frames = int(gp("stable_frames").value)
        self.conf_threshold = float(gp("conf_threshold").value)
        self.cooldown = float(gp("cooldown").value)
        self.linear_speed = float(gp("linear_speed").value)
        self.action_duration = float(gp("action_duration").value)
        self.kp = float(gp("angular_kp").value)
        self.w_max = float(gp("angular_max").value)
        self.tol = float(gp("angular_tol").value)

        self.pub_cmd = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pub_action = self.create_publisher(String, "/mnist_digit_control/action", 10)
        self.create_subscription(Int32, gp("digit_topic").value, self._on_digit, 10)
        self.create_subscription(Float32, gp("conf_topic").value, self._on_conf, 10)
        self.create_subscription(Pose, "/turtle1/pose", self._on_pose, 10)

        # 状态机：idle -> action -> cooldown -> idle
        self.state = "idle"
        self.state_until = 0.0
        self.history = []           # 最近 stable_frames 个识别数字
        self.confidence = 0.0
        self.last_executed = -1     # 上次触发的数字（防重复）
        self.miss_count = 0         # 连续未检测到数字的帧数
        self.pending = []           # 动作执行期间的待执行数字队列
        self.cooldown_until = 0.0
        self.action = None          # 当前动作字典
        self.yaw_accum = 0.0        # 累计转角
        self.yaw_prev = None
        self.on_target_ticks = 0
        self.pose = None
        self.executed_count = 0

        self.create_timer(1.0 / float(gp("control_hz").value), self.tick)
        self.get_logger().info("数字指令控制节点就绪，等待识别数字……")

    # ----------------------------------------------------------------- callbacks
    def _on_digit(self, msg: Int32) -> None:
        d = int(msg.data)
        if d < 0:
            self.miss_count += 1
            if self.miss_count >= 3:
                # 数字消失足够久，允许同一数字再次触发
                self.last_executed = -1
        else:
            self.miss_count = 0
        self.history.append(d)
        self.history = self.history[-self.stable_frames:]
        # 动作执行期间到达的稳定识别结果进入待执行队列，避免长动作期间丢指令
        if (self.state == "action" and d >= 0
                and len(self.history) >= self.stable_frames
                and all(x == d for x in self.history)
                and (not self.pending or self.pending[-1] != d)
                and (self.action is None or d != self.action["digit"])):
            self.pending.append(d)
            self.get_logger().info(f"动作执行中，数字 {d} 进入待执行队列")

    def _on_conf(self, msg: Float32) -> None:
        self.confidence = float(msg.data)

    def _on_pose(self, msg: Pose) -> None:
        self.pose = msg

    # ----------------------------------------------------------------- main loop
    def tick(self) -> None:
        now = time.monotonic()
        if self.state == "idle":
            self._try_trigger(now)
        elif self.state == "action":
            if not self._step_action(now):
                self._finish_action()
        elif self.state == "cooldown" and now >= self.state_until:
            if self.pending:
                # 优先执行动作期间排队到来的指令
                d = self.pending.pop(0)
                self._start_action(d, *ACTIONS[d])
            else:
                self.state = "idle"

    # ----------------------------------------------------------------- trigger
    def _try_trigger(self, now: float) -> None:
        if len(self.history) < self.stable_frames or now < self.cooldown_until:
            return
        d = self.history[-1]
        if d < 0 or any(x != d for x in self.history):
            return
        if self.confidence < self.conf_threshold:
            return
        if d == self.last_executed:
            # 同一数字持续出现只触发一次，数字消失或变化后才会再次触发
            return
        name, en, kind, arg = ACTIONS[d]
        self._start_action(d, name, en, kind, arg)

    # ----------------------------------------------------------------- actions
    def _start_action(self, digit: int, name: str, en: str, kind: str, arg: dict) -> None:
        self.last_executed = digit
        self.history = []           # 清空，要求下一次触发需要新的连续稳定段
        self.executed_count += 1
        self.state = "action"       # 进入动作执行态（stop 会在下方立即结束）
        self.action = {
            "digit": digit, "name": name, "kind": kind, "arg": arg,
            "t_start": time.monotonic(),
        }
        if kind == "rotate":
            self.yaw_accum = 0.0
            self.yaw_prev = self.pose.theta if self.pose is not None else None
            self.on_target_ticks = 0
        elif kind == "translate":
            self.action["t_end"] = self.action["t_start"] + self.action_duration
        self.get_logger().info(f"执行动作: 数字 {digit} -> {name}")
        self.pub_action.publish(String(data=en))
        if kind == "stop":
            self._publish_twist(0.0, 0.0, 0.0)
            self._finish_action()

    def _step_action(self, now: float) -> bool:
        """执行一步动作，返回 True 表示动作仍在进行。"""
        act = self.action
        if act["kind"] == "translate":
            if now >= act["t_end"]:
                self._publish_twist(0.0, 0.0, 0.0)
                return False
            vx = self.linear_speed * act["arg"].get("vx", 0.0)
            vy = self.linear_speed * act["arg"].get("vy", 0.0)
            self._publish_twist(vx, vy, 0.0)
            return True
        if act["kind"] == "rotate":
            target = math.radians(act["arg"]["deg"])
            if self.pose is None or self.yaw_prev is None:
                # 无位姿反馈时退化为开环定时旋转
                if now >= act["t_start"] + abs(target) / self.w_max:
                    self._publish_twist(0.0, 0.0, 0.0)
                    return False
                self._publish_twist(0.0, 0.0, self.w_max if target > 0 else -self.w_max)
                return True
            dtheta = wrap_angle(self.pose.theta - self.yaw_prev)
            self.yaw_prev = self.pose.theta
            self.yaw_accum += dtheta
            err = target - self.yaw_accum
            if abs(err) < self.tol:
                self.on_target_ticks += 1
                # 必须显式发布 0 速：turtlesim 会保持上一次速度继续转，
                # 若验证帧不发布指令，误差会在容差附近形成极限环震荡
                self._publish_twist(0.0, 0.0, 0.0)
                if self.on_target_ticks >= 3:
                    return False
            else:
                self.on_target_ticks = 0
                w = max(-self.w_max, min(self.w_max, self.kp * err))
                self._publish_twist(0.0, 0.0, w)
            return True
        return False

    def _finish_action(self) -> None:
        self.action = None
        self.state = "cooldown"
        self.state_until = time.monotonic() + self.cooldown
        self.pub_action.publish(String(data=""))

    def _publish_twist(self, vx: float, vy: float, wz: float) -> None:
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = wz
        self.pub_cmd.publish(twist)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DigitCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
