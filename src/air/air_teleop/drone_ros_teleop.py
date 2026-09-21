#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 键盘发布节点：drone_ros_teleop
# 作用：用 termios 原始模式直接读取终端按键，把按键翻译成 NED 机体坐标系下的
#       速度指令（geometry_msgs/Twist），并按 10 Hz 发布到 /drone/cmd_vel。
#       该节点不直接接触仿真器，只负责“键盘 -> ROS 消息”的转换。
#
# 为什么不用 pynput：虚拟机/SSH 终端里 pynput 捕获不到按键，按键会退化成普通
# 字符输入（屏幕上回显 wwwdwa 且不发布指令）。改为 tty.setraw(sys.stdin.fileno())
# 进入原始模式，仅依赖 Python 标准库（termios/tty/select/os）：
#   1. 关闭终端字符回显，按键不再打在屏幕上；
#   2. 主循环逐字符非阻塞监听（select + os.read），捕获到的键值直接转为小写，
#      无需按 Enter，即按即发（按下瞬间立即发布一次，终端上单行 \r 原地刷新
#      当前状态，10 Hz 定时器持续补发同一指令）；终端无法多键并发，
#      斜向飞行由复合按键（q/e/z/c）单独给出整组速度指令；
#   3. 原始模式下普通按键只有按下事件、没有松开事件，因此用“按住窗口”模拟：
#      每次按下都会刷新最近按键时刻，超过 HOLD_TIMEOUT（默认 0.4 s）没有新按键
#      就把最后指令速度沿线性衰减到 0（DECAY_TIME，默认 0.5 s），而不是瞬间
#      硬跳变，避免飞控频繁触发制动阻尼导致飞行抽搐；'k' 键强制急刹立即全零。
#   4. ESC / Ctrl+C 退出，退出前用 termios.tcsetattr 恢复终端原始属性。
#
# 坐标系：与桥接节点（drone_ros_node）约定一致，消息按 NED 机体坐标系解释：
#         x 前、y 右、z 指向地面；linear.z < 0 上升、angular.z > 0 右偏航。

import os
import select
import sys
import threading
import time

import rospy
from geometry_msgs.msg import Twist

try:
    import termios
    import tty
except ImportError:                 # Windows 没有 termios；本节点只支持 Linux/macOS 终端
    termios = None
    tty = None


class DroneTeleop:
    """原始模式读取按键并按 10 Hz 发布 /drone/cmd_vel 的 ROS 节点"""

    # ---- 标称速度：按键对照表与速度计算共用同一份参数 ----
    LINEAR_SPEED = 3.0                            # 前后标称线速度 3.0 m/s
    DIAG_SPEED = 2.5                              # 斜向飞行的横向分量 2.5 m/s
    VERTICAL_SPEED = 2.0                          # 垂直标称速度 2.0 m/s
    YAW_RATE = 0.5                                # 偏航角速度 0.5 rad/s（约 28.6 度/秒）
    HOLD_TIMEOUT = 0.4                            # 无新按键多久后开始减速（秒）
    DECAY_TIME = 0.5                              # 松键后速度线性衰减到 0 的时长（秒）
    READ_POLL = 0.05                              # 按键监听线程轮询间隔（秒）

    # ---- 按键映射表：内部键名 -> 功能说明（状态行显示用）----
    KEYMAP = {
        'w': '前进', 's': '后退',
        'a': '左移', 'd': '右移',
        'q': '左前', 'e': '右前',
        'z': '左后', 'c': '右后',
        'j': '左偏航', 'l': '右偏航',
        'space': '上升', 'x': '下降',
        'k': '急刹',
    }

    # ---- 按键 -> 完整速度指令 (vx, vy, vz, yaw_rate) ----
    # 机体坐标系：x 前、y 右、z 向下（NED）；终端无法多键并发，
    # 斜向飞行由复合按键（q/e/z/c）单独给出整组指令
    VELOCITY_MAP = {
        'w': (LINEAR_SPEED, 0.0, 0.0, 0.0),
        's': (-LINEAR_SPEED, 0.0, 0.0, 0.0),
        'a': (0.0, -LINEAR_SPEED, 0.0, 0.0),
        'd': (0.0, LINEAR_SPEED, 0.0, 0.0),
        'q': (LINEAR_SPEED, -DIAG_SPEED, 0.0, 0.0),
        'e': (LINEAR_SPEED, DIAG_SPEED, 0.0, 0.0),
        'z': (-LINEAR_SPEED, -DIAG_SPEED, 0.0, 0.0),
        'c': (-LINEAR_SPEED, DIAG_SPEED, 0.0, 0.0),
        'j': (0.0, 0.0, 0.0, -YAW_RATE),
        'l': (0.0, 0.0, 0.0, YAW_RATE),
        'space': (0.0, 0.0, -VERTICAL_SPEED, 0.0),
        'x': (0.0, 0.0, VERTICAL_SPEED, 0.0),
    }

    # ---- 原始模式按键字符 -> 内部键名（主循环已统一转为小写）----
    CHAR_TO_NAME = {
        'w': 'w', 's': 's', 'a': 'a', 'd': 'd',
        'q': 'q', 'e': 'e', 'z': 'z', 'c': 'c',
        'j': 'j', 'l': 'l', 'k': 'k', 'x': 'x',
        ' ': 'space',
        '\x03': 'exit',                           # Ctrl+C（原始模式下 ISIG 已关闭，需自行处理）
    }

    def __init__(self, rate=10.0):
        self.rate = max(0.1, rate)                # 防止 0 频率导致除零

        # 按键状态：按键监听线程写入、ROS 定时器线程读取
        self.lock = threading.Lock()
        self.last_key = None                      # 当前指令键（内部键名），None = 悬停/减速
        self.last_press = 0.0                     # 最近一次按键时刻（monotonic 时钟）
        self.decay_from = None                    # 衰减起点速度 (vx, vy, vz, yaw)，None = 无衰减
        self.decay_start = 0.0                    # 衰减开始时刻（monotonic 时钟）
        self.exit_requested = False               # 是否收到 ESC / Ctrl+C
        self.stopped = False                      # 安全收尾是否已完成
        self.raw_active = False                   # 是否已进入原始模式（决定退出时是否恢复）
        self.stdin_fd = None
        self.old_settings = None
        self.wakeup = threading.Event()           # 通知按键监听线程退出

        self.pub = rospy.Publisher('/drone/cmd_vel', Twist, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self.publish_cb)
        self.listener = threading.Thread(target=self._key_loop,
                                         name='termios-key-listener', daemon=True)
        rospy.on_shutdown(self.stop)

    # ---------------- 终端原始模式 ----------------

    def _enter_raw(self):
        """终端进入原始模式：关闭回显与行缓冲，退出时由 _restore_term 恢复"""
        if termios is None or tty is None:
            raise RuntimeError('termios/tty 仅支持 Linux/macOS 终端，'
                               '本节点无法在 Windows 原生终端运行')
        if not sys.stdin.isatty():
            raise RuntimeError('stdin 不是交互式终端，请直接在终端里运行本节点，'
                               '不要重定向输入（SSH 终端与虚拟机终端均可）')
        self.stdin_fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.stdin_fd)
        tty.setraw(self.stdin_fd)
        self.raw_active = True

    def _restore_term(self):
        """恢复终端原始属性（幂等，可被 stop 与 finally 重复调用）"""
        if self.raw_active and self.old_settings is not None:
            termios.tcsetattr(self.stdin_fd, termios.TCSADRAIN, self.old_settings)
            self.raw_active = False

    # ---------------- 按键监听 ----------------

    def _read_key(self):
        """非阻塞读一个按键字符：无输入返回 None，stdin 关闭返回 ''"""
        try:
            if not select.select([self.stdin_fd], [], [], self.READ_POLL)[0]:
                return None
            data = os.read(self.stdin_fd, 1)
            if not data:
                return ''                         # stdin 被关闭
            return data.decode('latin-1')         # 按键均为单字节 ASCII/控制字符
        except OSError:
            return ''

    def _drain_esc_sequence(self):
        """ESC 后再探测 50 ms：有后续字节说明是方向键等转义序列（忽略），否则是退出键"""
        deadline = time.monotonic() + 0.05
        while time.monotonic() < deadline:
            if not select.select([self.stdin_fd], [], [], 0.01)[0]:
                continue
            if os.read(self.stdin_fd, 64):
                return True
        return False

    def _on_key(self, key):
        """按下有效按键：记录为当前指令键、立即发布一次并原地刷新状态行"""
        if key in ('esc', 'exit'):                # ESC / Ctrl+C → 请求退出
            self.exit_requested = True
            return
        name = self.CHAR_TO_NAME.get(key)
        if name is None:                          # 无效按键（Enter、方向键残片等）忽略
            return
        with self.lock:
            if name == 'k':                       # 强制急刹：清空指令，立即全零
                self.last_key = None
                self.decay_from = None
                vel = (0.0, 0.0, 0.0, 0.0)
            else:
                self.last_key = name              # 新按键覆盖旧按键，指令唯一来源
                self.last_press = time.monotonic()
                self.decay_from = None            # 取消进行中的衰减，恢复全速指令
                vel = self.VELOCITY_MAP[name]
            msg = self.build_twist(vel)
        self.pub.publish(msg)                     # 即按即发，不等 10 Hz 定时器
        self._show_status(msg, self.KEYMAP.get(name, '-'))

    def _key_loop(self):
        """按键监听主循环：逐字符非阻塞读 stdin，直接转小写后分发"""
        while not self.wakeup.is_set():
            key = self._read_key()
            if key is None:
                continue
            if key == '':                         # stdin 被关闭，主动退出
                self.exit_requested = True
                break
            key = key.lower()                     # 捕获到的键值直接转为小写
            if key == '\x1b':                     # ESC：退出键或转义序列前缀
                if not self._drain_esc_sequence():
                    self._on_key('esc')
                continue
            self._on_key(key)

    # ---------------- 消息发布 ----------------

    def build_twist(self, vel):
        """把速度四元组 (vx, vy, vz, yaw_rate) 写入 NED 机体坐标系 Twist 消息

        机体坐标系：linear.x 前、linear.y 右、linear.z 下（负值上升）；
        angular.z 偏航角速度弧度/秒，正值右偏航（顺时针，从上方看）。
        """
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z = vel
        return msg

    @staticmethod
    def _show_status(msg, label):
        """单行原地刷新状态：\r 回到行首覆盖上一条，终端不再逐帧换行刷屏"""
        print(f'\r[当前状态] 速度: vx={msg.linear.x:.1f}, vy={msg.linear.y:.1f}, '
              f'vz={msg.linear.z:.1f}, yaw={msg.angular.z:.1f} | 按键: {label}',
              end='', flush=True)

    def publish_cb(self, _event):
        """定时回调：按键指令全速下发；松键超时后线性衰减到 0；ESC/Ctrl+C 安全退出"""
        if self.exit_requested:
            # 不在按键监听线程中直接 shutdown（on_shutdown 会 join 监听线程自身）
            rospy.signal_shutdown('收到 ESC/Ctrl+C，退出键盘遥控')
            return
        with self.lock:
            now = time.monotonic()
            # 原始模式探测不到松开事件：超过 HOLD_TIMEOUT 没有新按键就以最后
            # 指令速度为起点开始线性衰减（等效于松键），衰减结束持续发布全零悬停
            if self.last_key is not None and now - self.last_press > self.HOLD_TIMEOUT:
                self.decay_from = self.VELOCITY_MAP[self.last_key]
                self.decay_start = now
                self.last_key = None
            if self.last_key is not None:
                vel = self.VELOCITY_MAP[self.last_key]
                label = self.KEYMAP[self.last_key]
            elif self.decay_from is not None:
                progress = (now - self.decay_start) / self.DECAY_TIME
                if progress >= 1.0:
                    self.decay_from = None        # 衰减结束，进入原地悬停
                    vel = (0.0, 0.0, 0.0, 0.0)
                    label = '-'
                else:
                    # 线性插值衰减；分量小于 1e-6 视为 0，避免浮点尾数
                    # 让末帧残留极小速度，保证衰减收尾干净
                    vel = tuple(0.0 if abs(v) < 1e-6 else v
                                for v in (k * (1.0 - progress)
                                          for k in self.decay_from))
                    label = '减速中'
            else:
                vel = (0.0, 0.0, 0.0, 0.0)
                label = '-'
            msg = self.build_twist(vel)
        self.pub.publish(msg)
        self._show_status(msg, label)

    # ---------------- 生命周期 ----------------

    @staticmethod
    def print_help():
        """启动时打印按键对照表"""
        print('=' * 60)
        print('        AirSim 无人机键盘遥控（termios 原始模式）')
        print('=' * 60)
        print('  W / S       前进 / 后退        (vx = ±%.1f)' % DroneTeleop.LINEAR_SPEED)
        print('  A / D       左移 / 右移        (vy = -/+%.1f)' % DroneTeleop.LINEAR_SPEED)
        print('  Q / E       左前 / 右前        (vx = +%.1f, vy = -/+%.1f)'
              % (DroneTeleop.LINEAR_SPEED, DroneTeleop.DIAG_SPEED))
        print('  Z / C       左后 / 右后        (vx = -%.1f, vy = -/+%.1f)'
              % (DroneTeleop.LINEAR_SPEED, DroneTeleop.DIAG_SPEED))
        print('  J / L       原地左偏航 / 右偏航 (yaw = -/+%.1f rad/s)' % DroneTeleop.YAW_RATE)
        print('  Space / X   垂直上升 / 垂直下降 (vz = -/+%.1f)' % DroneTeleop.VERTICAL_SPEED)
        print('  K           强制急刹          (全 0 速度)')
        print('  ESC/Ctrl+C  退出键盘遥控（桥接节点 0.5 s 超时后自动悬停）')
        print('-' * 60)
        print('  终端已进入原始模式：关闭回显、无需回车、即按即发')
        print('  复合按键 q/e/z/c 提供斜向飞行，新按键覆盖旧按键')
        print('  %.1f s 无新按键后速度线性衰减 %.1f s 至 0（平滑悬停，不硬刹）'
              % (DroneTeleop.HOLD_TIMEOUT, DroneTeleop.DECAY_TIME))
        print('=' * 60)

    def start(self):
        """进入原始模式、启动按键监听线程并进入 ROS 循环"""
        self.print_help()
        try:
            self._enter_raw()
        except RuntimeError as exc:
            rospy.logerr('%s', exc)
            rospy.signal_shutdown('无法进入原始模式')
            return
        self.listener.start()
        rospy.loginfo('键盘节点已启动：原始模式监听按键，按 %.1f Hz 发布 /drone/cmd_vel',
                      self.rate)
        rospy.spin()

    def stop(self):
        """安全收尾：退出按键监听并恢复终端原始属性（幂等）"""
        if self.stopped:
            return
        self.stopped = True
        self.wakeup.set()
        self._restore_term()
        print()                                   # 收尾换行，避免提示符粘在状态行末尾
        rospy.loginfo('键盘节点已退出，终端属性已恢复，停止发布速度指令')


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description='AirSim 无人机键盘发布节点：键盘 -> /drone/cmd_vel')
    parser.add_argument('--rate', type=float, default=None, help='发布频率（Hz），默认 10')
    args, _ = parser.parse_known_args()               # 兼容 ROS 参数重映射传入的 _xxx:=yyy

    rospy.init_node('drone_ros_teleop')
    rate = args.rate if args.rate is not None else rospy.get_param('~rate', 10.0)

    teleop = DroneTeleop(rate=rate)
    try:
        teleop.start()
    except KeyboardInterrupt:
        pass
    finally:
        teleop.stop()


if __name__ == '__main__':
    main()
