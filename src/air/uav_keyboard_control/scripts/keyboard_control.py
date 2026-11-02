#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘控制节点：终端按键 -> /uav/cmd_vel（机体系速度指令）.

按键映射（机体系：x 前 / y 左 / z 上；速度单位为 m/s，偏航角速度为 rad/s）：
    W/S   前进 / 后退
    A/D   左移 / 右移
    R/F   上升 / 下降
    Q/E   右转 / 左转（Q=+yaw_rate 机头右转，遵循 AirSim 机体系 yaw_rate 约定）
    组合键 可同时按住多个方向键（W+A 斜向、W+Q 边走边转），各轴分量叠加
    其它键 / 松开    悬停（速度按加速度上限平滑回零，而不是瞬间跳变）
    ESC 或 Ctrl-C     退出（退出前补发一条零速指令，避免无人机保持最后速度）

发布：
    /uav/cmd_vel   geometry_msgs/Twist   机体系速度指令（默认 30 Hz）

依赖：
    需要 carlair_ros_bridge 已启动并订阅 /uav/cmd_vel；本模块只发指令，不直连仿真器。
    这正是任务①"无人机仿真 + 键盘控制"的分工：仿真/坐标换算在桥接层，键盘只管下发。

实现要点（详见 docs/air/uav_keyboard_control.md 第 4 节）：
    1) 终端只送"按下"的字节，没有"抬起"事件。因此"是否按住"只能用
       "最近 hold_timeout 内是否还收到过这个键"来判断：按住时终端自动重复
       （autorepeat）会持续送字节，停手后不再送，超时即判松开。
    2) 多个键同时按住时，各轴分量相加：W+A -> (vx=+h, vy=+h, 0, 0)；
       W+S 这种互斥组合自然抵消为 0（悬停）。
    3) 发布前经加速度限幅：速度按 accel_* 斜坡逼近目标值，桥接层收到的是一条
       连续变化的指令序列，而不是从 0 直接跳到 2 m/s 的阶跃。
"""
from __future__ import annotations

import os
import select
import sys
import time

import rospy
from geometry_msgs.msg import Twist

# 按键 -> (轴, 符号)。符号 +1 表示该轴正方向，-1 表示负方向。
KEY_MAP = {
    "w": ("forward", +1), "s": ("forward", -1),
    "a": ("left", +1), "d": ("left", -1),
    "r": ("up", +1), "f": ("up", -1),
    "q": ("yaw", +1), "e": ("yaw", -1),
}

# 按键保持判定的默认时长（s）。终端自动重复通常在 25~40 ms 一个字节，
# 150 ms 相当于连丢 4 帧才判松开，既不会因偶发丢帧抖动，松手后也不会拖太久。
HOLD_TIMEOUT = 0.15
# 单帧最大积分时长（s）：终端若被阻塞（例如用户拖动窗口），避免一帧把速度积分到很大。
MAX_LOOP_DT = 0.1

ESCAPE_TIMEOUT = 0.02

HELP_TEXT = """
============================================================
  无人机键盘控制（机体系速度，发布到 /uav/cmd_vel）
------------------------------------------------------------
  W / S     前进 / 后退
  A / D     左移 / 右移
  R / F     上升 / 下降
  Q / E     右转 / 左转
  组合键    可同时按（W+A 斜飞、W+Q 边走边转）
  松开按键   平滑减速到悬停
  ESC / Ctrl-C 退出
============================================================
"""


def _axis(key):
    """键 -> (轴, 符号)；未知键/空键返回 None."""
    if not key:
        return None
    return KEY_MAP.get(key.lower())


def merge_target(keys, horiz_speed=2.0, vert_speed=1.0, yaw_rate=1.0):
    """一组成员按键 -> 机体系目标速度 (vx, vy, vz, wz).

    纯函数：不依赖 ROS / 终端，便于单元测试。未知键忽略（例如方向键序列的残字节），
    互斥方向同时按住时互相抵消。
    """
    vx = vy = vz = wz = 0.0
    for key in keys:
        hit = _axis(key)
        if hit is None:
            continue
        axis, sign = hit
        if axis == "forward":
            vx += horiz_speed * sign
        elif axis == "left":
            vy += horiz_speed * sign
        elif axis == "up":
            vz += vert_speed * sign
        elif axis == "yaw":
            wz += yaw_rate * sign
    return (vx, vy, vz, wz)


def key_to_cmd(key, horiz_speed=2.0, vert_speed=1.0, yaw_rate=1.0):
    """单个按键 -> 机体系速度 (vx, vy, vz, wz)；无按键返回全零；未知键返回 None.

    保留的便捷封装（单键语义 = 组合键集合里只有一个元素），纯函数。
    """
    if not key:
        return (0.0, 0.0, 0.0, 0.0)
    if _axis(key) is None:
        return None
    return merge_target([key], horiz_speed, vert_speed, yaw_rate)


def ramp_towards(current, target, max_delta):
    """一步斜坡：current 以最大步长 max_delta 逼近 target，不越过目标."""
    if max_delta <= 0:          # 加速度被设为 0 -> 不平滑，直接到位
        return target
    if target > current:
        return min(target, current + max_delta)
    if target < current:
        return max(target, current - max_delta)
    return target


class KeyTracker(object):
    """按键保持状态：用"最近一次收到该键的时间"表示是否按住.

    终端没有按键抬起事件，只能这样做：按住时自动重复不断送字节，松手后字节停止，
    超过 hold_timeout 未再收到即视为松开。
    """

    def __init__(self, hold_timeout=HOLD_TIMEOUT):
        self.hold_timeout = float(hold_timeout)
        if self.hold_timeout <= 0:          # 参数填 0 视为缺省，避免"按一下就永远松开"
            self.hold_timeout = HOLD_TIMEOUT
        self._seen = {}

    def press(self, key, now):
        """记录一次按键字节；未知键/转义序列残字节不参与运动，直接忽略."""
        if _axis(key) is not None:
            self._seen[key.lower()] = now

    def active(self, now):
        """返回当前仍按住的键（按字母序，便于测试与日志稳定）."""
        expired = [k for k, t in self._seen.items() if now - t > self.hold_timeout]
        for k in expired:
            del self._seen[k]
        return tuple(sorted(self._seen))

    def release_all(self):
        self._seen.clear()


class VelocitySmoother(object):
    """按加速度上限把速度斜坡逼近目标值.

    accel <= 0 表示该轴不限制（瞬时到位），保留"要原汁原味阶跃"的用法。
    """

    def __init__(self, accel_horiz=4.0, accel_vert=2.0, accel_yaw=4.0):
        self.accel = (float(accel_horiz), float(accel_horiz),
                      float(accel_vert), float(accel_yaw))
        self.cmd = (0.0, 0.0, 0.0, 0.0)

    def update(self, target, dt):
        """推进一个控制周期，返回实际下发的 (vx, vy, vz, wz)."""
        dt = max(0.0, min(float(dt), MAX_LOOP_DT))
        if dt <= 0.0:
            # 首帧 / 时钟未推进：不做积分，否则 accel*dt=0 会被当成"不限制"
            return self.cmd
        self.cmd = tuple(
            ramp_towards(cur, tgt, accel * dt)
            for cur, tgt, accel in zip(self.cmd, target, self.accel)
        )
        return self.cmd

    def reset(self):
        self.cmd = (0.0, 0.0, 0.0, 0.0)


def consume_escape(reader):
    """ESC 之后若紧跟 CSI/SS3 序列（方向键、Home、F1 等），把整段吃掉并返回 True.

    单独按下 ESC 返回 False（调用方据此退出）。这样按方向键不会误退出，
    序列残字节（如 "[A"）也不会被当成运动键。
    """
    nxt = reader.get_key(timeout=ESCAPE_TIMEOUT)
    if nxt not in ("[", "O"):
        return False
    while True:
        ch = reader.get_key(timeout=ESCAPE_TIMEOUT)
        if ch is None or "@" <= ch <= "~":   # 超时兜底 / CSI 终止字节
            return True


class RawKeyReader(object):
    """Linux 终端原始模式按键读取（不回显、不需回车）.

    roslaunch 启动的节点 stdin 指向 /dev/null（ROS 的已知限制，这也是
    teleop_twist_keyboard 通常用 rosrun 跑的原因）。为满足"支持 launch 启动"，
    这里优先直接打开控制终端 ``/dev/tty`` 读键；没有控制终端时回退到 stdin。
    用 ``with RawKeyReader() as reader:`` 保证退出/异常时恢复终端设置。
    """

    def __init__(self):
        self._close_fd = False
        try:
            self._fd = os.open("/dev/tty", os.O_RDONLY)
            self._close_fd = True
        except OSError:
            self._fd = sys.stdin.fileno()

    def __enter__(self):
        import termios
        import tty
        self._old = termios.tcgetattr(self._fd)
        tty.setraw(self._fd)
        return self

    def __exit__(self, *exc):
        import termios
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
        if self._close_fd:
            os.close(self._fd)

    def get_key(self, timeout=0.05):
        """非阻塞读一个字符；超时或无输入返回 None."""
        r, _, _ = select.select([self._fd], [], [], timeout)
        if not r:
            return None
        ch = os.read(self._fd, 1)
        if not ch:
            return None
        try:
            return ch.decode("utf-8")
        except UnicodeDecodeError:
            return None


class KeyboardController(object):
    """读取按键并把速度指令发布到 /uav/cmd_vel."""

    def __init__(self):
        self.horiz = float(rospy.get_param("control/horiz_speed", 2.0))
        self.vert = float(rospy.get_param("control/vert_speed", 1.0))
        self.yaw = float(rospy.get_param("control/yaw_rate", 1.0))
        self.hold_timeout = float(rospy.get_param("control/key_hold_timeout", HOLD_TIMEOUT))
        topic = rospy.get_param("topic/cmd_vel", "/uav/cmd_vel")
        rate_hz = float(rospy.get_param("rate/publish_hz", 30.0))
        self.pub = rospy.Publisher(topic, Twist, queue_size=1)
        self.rate = rospy.Rate(rate_hz)
        self.tracker = KeyTracker(self.hold_timeout)
        self.smoother = VelocitySmoother(
            rospy.get_param("control/accel_horiz", 4.0),
            rospy.get_param("control/accel_vert", 2.0),
            rospy.get_param("control/accel_yaw", 4.0),
        )
        self._last_time = None

    # ------------------------------------------------------------------ 核心
    def build_twist(self, cmd):
        """(vx, vy, vz, wz) -> geometry_msgs/Twist."""
        vx, vy, vz, wz = cmd
        t = Twist()
        t.linear.x = vx
        t.linear.y = vy
        t.linear.z = vz
        t.angular.z = wz
        return t

    def step_once(self, key, now=None, dt=None):
        """把一次按键（可能为 None）推进一个控制周期，返回要发布的 Twist.

        独立成函数、时间可由外部注入，便于无 ROS 的单元测试。
        """
        now = time.monotonic() if now is None else float(now)
        if dt is None:
            dt = 0.0 if self._last_time is None else now - self._last_time
        self._last_time = now

        self.tracker.press(key, now)
        target = merge_target(self.tracker.active(now), self.horiz, self.vert, self.yaw)
        return self.build_twist(self.smoother.update(target, dt))

    def stop(self):
        """立即补发零速（退出/异常路径调用），不再等桥接层的超时保护."""
        self.tracker.release_all()
        self.smoother.reset()
        try:
            self.pub.publish(self.build_twist((0.0, 0.0, 0.0, 0.0)))
        except Exception as exc:  # noqa: BLE001 - 退出路径上不能再抛
            rospy.logwarn("uav_keyboard_control: 退出时补发零速失败: %s", exc)

    # ------------------------------------------------------------------ 主循环
    def spin(self):
        print(HELP_TEXT)
        try:
            with RawKeyReader() as reader:
                while not rospy.is_shutdown():
                    now = time.monotonic()
                    key = reader.get_key(timeout=0.03)
                    if key == "\x03":            # Ctrl-C
                        print("\n已退出键盘控制")
                        break
                    if key == "\x1b":            # ESC：先排除方向键等转义序列
                        if consume_escape(reader):
                            continue
                        print("\n已退出键盘控制")
                        break
                    self.pub.publish(self.step_once(key, now=now))
                    self.rate.sleep()
        finally:
            self.stop()


def main():
    rospy.init_node("uav_keyboard_control", anonymous=False)
    node = KeyboardController()
    try:
        node.spin()
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C，已退出")


if __name__ == "__main__":
    main()
