#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
teleop_keyboard.py —— 无人车键盘运动控制
==========================================

【任务书对应】任务 (1)：
    「对人形机器人/肌肉骨骼人/车辆/水下机器人/无人机进行物理仿真，
      通过键盘对机器人进行运动控制（5 分）」

【功能说明】
    在 Gazebo 物理仿真环境中，通过键盘实时控制无人车的线速度与角速度，
    实现前进、后退、左转、右转、加速、减速与紧急停车。

【键盘映射】
       w : 前进（线速度 +）          s : 后退（线速度 -）
       a : 左转（角速度 +）          d : 右转（角速度 -）
       q : 前进+左转                 e : 前进+右转
       z : 后退+左转                 c : 后退+右转
    空格 : 紧急停止（速度归零）
       + : 提高最大速度              - : 降低最大速度
       x : 退出程序

【运动学原理】
    本车为差速驱动 (Differential Drive) 底盘，其运动学模型为：

        线速度      v = (v_r + v_l) / 2
        角速度      ω = (v_r - v_l) / L

    其中 v_l、v_r 分别为左右轮线速度，L 为轮间距（本车 0.335 m）。

    键盘输入并非直接设定 v 与 ω，而是设定「速度增量」：
        每次按键后  v ← clamp(v + dv, -v_max, +v_max)
                    ω ← clamp(ω + dω, -ω_max, +ω_max)

    这种「增量式」控制相比「直接设定」更接近真实遥控器的体验：
    松开按键后车辆保持当前速度继续行驶，需要主动按键才能改变状态。

【与 gazebo_ros_diff_drive 的接口】
    本节点发布 geometry_msgs/Twist 到 /cmd_vel 话题。
    gazebo_ros_diff_drive 插件订阅该话题后，通过以下公式解算左右轮角速度：

        ω_l = (v - ω·L/2) / r
        ω_r = (v + ω·L/2) / r

    其中 r 为车轮半径（本车 0.0975 m）。

【作者】OpenHUTB Car Team
"""

from __future__ import print_function

import sys
import select
import termios
import tty

import rospy
from geometry_msgs.msg import Twist


# ============================================================================
# 速度参数
# ============================================================================

#: 单次按键的线速度增量 (m/s)
LINEAR_STEP = 0.1

#: 单次按键的角速度增量 (rad/s)
ANGULAR_STEP = 0.2

#: 线速度上限 (m/s)。仿真中过高的速度会导致激光雷达来不及响应而撞墙。
LINEAR_MAX = 0.8

#: 角速度上限 (rad/s)
ANGULAR_MAX = 1.2

#: 提示信息刷新周期 (次)。每 N 次循环打印一次当前速度，避免刷屏。
INFO_PRINT_EVERY = 10


HELP_TEXT = """
============================================================
          OpenHUTB 无人车 —— 键盘运动控制
============================================================
   移动控制                     速度调节
   ----------------------       ----------------------
     w : 前进                     + : 提高最大速度
     s : 后退                     - : 降低最大速度
     a : 左转
     d : 右转                     其他
     q : 前进 + 左转              ------
     e : 前进 + 右转               空格 : 紧急停止
     z : 后退 + 左转                 x  : 退出程序
     c : 后退 + 右转
   ----------------------
   注意：松开按键后车辆保持当前速度，需按空格停车。
============================================================
"""


class TeleopKeyboardNode(object):
    """键盘遥控节点。

    通过读取终端原始输入 (raw mode) 实现无回车的即时按键响应。
    使用非阻塞 select 轮询，保证 update() 能被主循环以固定频率调用，
    不会因等待按键而阻塞整个控制系统。
    """

    def __init__(self, watchdog):
        """
        :param watchdog: VelocityWatchdog 实例，用于刷新安全计时并输出指令
        """
        self._watchdog = watchdog

        # 当前速度状态
        self._linear = 0.0
        self._angular = 0.0

        # 当前速度上限（可运行时调节）
        self._linear_max = LINEAR_MAX
        self._angular_max = ANGULAR_MAX

        # 终端设置备份
        self._settings = None
        self._tick = 0
        self._running = False

        # 判断是否有可用终端。若无终端（例如通过 launch 后台启动），
        # 则退化为「匀速前进」模式，保证仿真仍可运行并产出轨迹数据。
        self._has_tty = sys.stdin.isatty()
        if not self._has_tty:
            rospy.logwarn(
                "[teleop] 未检测到交互式终端，自动切换为【自动巡航模式】："
                "车辆将持续以 %.2f m/s 前进", LINEAR_STEP * 2
            )

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    def start(self):
        """启用终端原始模式。"""
        self._running = True
        if self._has_tty:
            try:
                self._settings = termios.tcgetattr(sys.stdin)
                tty.setraw(sys.stdin.fileno())
            except termios.error as exc:
                rospy.logwarn("[teleop] 无法设置终端原始模式：%s", exc)
                self._has_tty = False

            print(HELP_TEXT)

    def stop(self):
        """恢复终端设置。"""
        self._running = False
        if self._has_tty and self._settings is not None:
            try:
                termios.tcsetattr(
                    sys.stdin, termios.TCSADRAIN, self._settings
                )
            except termios.error:
                pass

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------

    def update(self):
        """每次循环调用：读取按键 → 更新速度 → 发布指令。"""
        if not self._running:
            return

        if self._has_tty:
            key = self._read_key()
            if key is not None:
                self._handle_key(key)
        else:
            # 无终端时的自动巡航：持续微调以产生可见轨迹
            if self._tick == 0:
                self._linear = min(LINEAR_STEP * 2, self._linear_max)

        self._tick += 1

        # 周期性打印当前状态
        if self._has_tty and self._tick % INFO_PRINT_EVERY == 0:
            self._print_status()

        self._publish()

    # ------------------------------------------------------------------
    # 内部方法
    # ------------------------------------------------------------------

    def _read_key(self):
        """非阻塞读取一个字符。无输入时返回 None。"""
        try:
            dr, _, _ = select.select([sys.stdin], [], [], 0)
            if dr:
                return sys.stdin.read(1)
        except (select.error, IOError):
            return None
        return None

    def _handle_key(self, key):
        """按键映射到速度变化。"""
        # ---- 移动控制 ----
        if key == 'w':
            self._linear += LINEAR_STEP
        elif key == 's':
            self._linear -= LINEAR_STEP
        elif key == 'a':
            self._angular += ANGULAR_STEP
        elif key == 'd':
            self._angular -= ANGULAR_STEP
        elif key == 'q':
            self._linear += LINEAR_STEP
            self._angular += ANGULAR_STEP
        elif key == 'e':
            self._linear += LINEAR_STEP
            self._angular -= ANGULAR_STEP
        elif key == 'z':
            self._linear -= LINEAR_STEP
            self._angular += ANGULAR_STEP
        elif key == 'c':
            self._linear -= LINEAR_STEP
            self._angular -= ANGULAR_STEP

        # ---- 紧急停止 ----
        elif key == ' ':
            self._linear = 0.0
            self._angular = 0.0
            rospy.loginfo("[teleop] 紧急停止")

        # ---- 速度上限调节 ----
        elif key == '+' or key == '=':
            self._linear_max = min(self._linear_max + LINEAR_STEP, 2.0)
            rospy.loginfo("[teleop] 线速度上限 → %.2f m/s", self._linear_max)
        elif key == '-':
            self._linear_max = max(self._linear_max - LINEAR_STEP, LINEAR_STEP)
            rospy.loginfo("[teleop] 线速度上限 → %.2f m/s", self._linear_max)

        # ---- 退出 ----
        elif key == 'x' or key == '\x03':  # Ctrl-C
            rospy.loginfo("[teleop] 收到退出指令")
            rospy.signal_shutdown("用户通过键盘退出")

        # ---- 限幅 ----
        self._linear = max(-self._linear_max,
                           min(self._linear, self._linear_max))
        self._angular = max(-self._angular_max,
                            min(self._angular, self._angular_max))

    def _publish(self):
        """构造并发布 Twist 消息。"""
        cmd = Twist()
        cmd.linear.x = self._linear
        cmd.angular.z = self._angular
        self._watchdog.feed(cmd)

    def _print_status(self):
        """打印当前速度状态（单行刷新）。"""
        sys.stdout.write(
            "\r  [速度]  线速度 v = %+.2f m/s   角速度 ω = %+.2f rad/s   "
            "上限 %.2f | %.2f   " % (
                self._linear, self._angular,
                self._linear_max, self._angular_max
            )
        )
        sys.stdout.flush()
