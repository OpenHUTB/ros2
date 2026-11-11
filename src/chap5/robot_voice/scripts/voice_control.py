#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
voice_control.py - 语音命令运动控制节点（兼容离线/无声卡环境）。

通过 /voice_cmd 话题（std_msgs/String）或交互式终端接收语音命令：
    "向前" / "向后" / "向左" / "向右" / "停止"
（同时兼容英文 forward / backward / left / right / stop 与常见同义词），
映射为两轮差速底盘 /cmd_vel 的 Twist 指令；收到命令后触发语音合成（TTS）反馈，
默认播报文本为 "太阳当空照，花儿对我笑"。

话题接口：
    订阅  voice_cmd    (std_msgs/String, launch 中重映射为 /voice_cmd)
    发布  /cmd_vel     (geometry_msgs/Twist)
    发布  ~tts_topic   (std_msgs/String, 供真实语音合成 SDK 节点订阅)

TTS 兼容策略（~tts_backend 参数）：
    log      —— 离线/无声卡环境默认：仅 rospy 日志输出 + 发布到 ~tts_topic
    espeak   —— Linux espeak 命令行合成（需 apt install espeak）
    festival —— Linux festival 命令行合成
    say      —— macOS 自带 say
    sdk      —— 交由已有讯飞 SDK 节点（tts_subscribe.cpp）订阅 ~tts_topic 播报
"""

from __future__ import print_function

import subprocess
import threading

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class VoiceControl(object):
    # 命令 -> (线速度方向, 角速度方向)，实际速度再乘以对应档位
    COMMANDS = {
        u"向前": (1.0, 0.0),
        u"前进": (1.0, 0.0),
        u"向后": (-1.0, 0.0),
        u"后退": (-1.0, 0.0),
        u"向左": (0.0, 1.0),
        u"左转": (0.0, 1.0),
        u"向右": (0.0, -1.0),
        u"右转": (0.0, -1.0),
        u"停止": (0.0, 0.0),
        u"停下": (0.0, 0.0),
        u"停": (0.0, 0.0),
        "forward": (1.0, 0.0),
        "backward": (-1.0, 0.0),
        "left": (0.0, 1.0),
        "right": (0.0, -1.0),
        "stop": (0.0, 0.0),
    }

    def __init__(self):
        # ---------- 参数 ----------
        self.linear_speed = rospy.get_param("~linear_speed", 0.35)   # 线速度档位 m/s
        self.angular_speed = rospy.get_param("~angular_speed", 1.2)  # 角速度档位 rad/s
        self.move_duration = rospy.get_param("~move_duration", 1.5)   # 运动时长 s, 0=保持到停止
        self.tts_backend = rospy.get_param("~tts_backend", "log")     # log/espeak/festival/say/sdk
        self.tts_text = rospy.get_param("~tts_text", u"太阳当空照，花儿对我笑")
        self.enable_terminal = rospy.get_param("~enable_terminal", True)

        # ---------- 话题 ----------
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        self.tts_pub = rospy.Publisher("~tts_topic", String, queue_size=5)
        self.voice_sub = rospy.Subscriber("voice_cmd", String,
                                          self.on_voice_cmd, queue_size=5)

        self._twist = Twist()
        self._stop_timer = None

        # 定时发布当前速度，保证差速底盘持续响应（约 10Hz）
        self._pub_timer = rospy.Timer(rospy.Duration(0.1), self._publish)

        # ---------- 交互式终端 ----------
        if self.enable_terminal:
            t = threading.Thread(target=self._terminal_loop)
            t.daemon = True
            t.start()
            rospy.loginfo("终端输入已开启，可直接输入：向前 / 向后 / 向左 / 向右 / 停止")

        rospy.loginfo("voice_control started, tts_backend=%s" % self.tts_backend)

    def on_voice_cmd(self, msg):
        self._execute(msg.data.strip())

    def _execute(self, cmd):
        if not cmd:
            return

        key = cmd.lower()
        if key not in self.COMMANDS:
            rospy.logwarn("未识别的语音指令: %s" % cmd)
            self._speak(u"未识别的指令：" + cmd)
            return

        linear_dir, angular_dir = self.COMMANDS[key]
        linear = linear_dir * self.linear_speed
        angular = angular_dir * self.angular_speed

        # 停止：立即清零并取消自动停止定时器
        if linear_dir == 0.0 and angular_dir == 0.0:
            self._cancel_stop_timer()
            self._set_velocity(0.0, 0.0)
            rospy.loginfo("语音指令 -> 停止")
            self._speak(self.tts_text)
            return

        self._set_velocity(linear, angular)
        rospy.loginfo("语音指令 '%s' -> v=%.2f  w=%.2f" % (cmd, linear, angular))
        self._speak(self.tts_text)

        # 到达设定时长后自动停止
        if self.move_duration > 0:
            self._cancel_stop_timer()
            self._stop_timer = rospy.Timer(rospy.Duration(self.move_duration),
                                           self._auto_stop, oneshot=True)

    def _set_velocity(self, linear, angular):
        self._twist.linear.x = linear
        self._twist.linear.y = 0.0
        self._twist.linear.z = 0.0
        self._twist.angular.x = 0.0
        self._twist.angular.y = 0.0
        self._twist.angular.z = angular

    def _publish(self, event):
        self.cmd_pub.publish(self._twist)

    def _auto_stop(self, event):
        rospy.loginfo("运动时长已到，自动停止")
        self._set_velocity(0.0, 0.0)

    def _cancel_stop_timer(self):
        if self._stop_timer is not None:
            self._stop_timer.shutdown()
            self._stop_timer = None

    def _speak(self, text):
        # 始终发布到 ~tts_topic，供真实语音合成 SDK 节点（如 tts_subscribe.cpp）订阅
        try:
            self.tts_pub.publish(String(data=text))
        except Exception:
            pass

        backend = self.tts_backend
        try:
            if backend == "espeak":
                subprocess.Popen(["espeak", "-v", "zh", text])
            elif backend == "festival":
                p = subprocess.Popen(["festival", "--tts"],
                                     stdin=subprocess.PIPE)
                p.communicate(text.encode("utf-8"))
            elif backend == "say":
                subprocess.Popen(["say", text])
            else:  # log / sdk
                rospy.loginfo("[TTS] %s" % text)
        except Exception as e:
            rospy.logwarn("tts backend '%s' 调用失败(%s)，降级为日志输出: %s" %
                          (backend, e, text))

    @staticmethod
    def _read_line():
        # Python2 使用 raw_input，Python3 使用 input
        try:
            import builtins  # noqa: F401 仅 Python3 存在
            read = input
        except ImportError:
            read = raw_input  # noqa: F821
        return read()

    def _terminal_loop(self):
        while not rospy.is_shutdown():
            try:
                line = self._read_line().strip()
            except (EOFError, IOError, KeyboardInterrupt):
                # 无 TTY / 输入流关闭时退出终端线程
                break
            if line:
                self._execute(line)

    def cleanup(self):
        self._cancel_stop_timer()
        self._set_velocity(0.0, 0.0)
        try:
            self.cmd_pub.publish(self._twist)
        except Exception:
            pass


if __name__ == "__main__":
    rospy.init_node("voice_control")
    node = VoiceControl()
    rospy.on_shutdown(node.cleanup)
    rospy.spin()
