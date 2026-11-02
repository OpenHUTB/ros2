#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手势指令 -> ``carlair_ros_bridge`` 的执行节点。

数据流
------
::

    gesture_control_drone（感知）            本模块（翻译/执行）
    /gesture_command  ────────────────►  GestureBridgeNode
                                            ├─ /uav/action  (std_msgs/String)  takeoff|land|hover
                                            ├─ /uav/cmd_vel (geometry_msgs/Twist) 机体系速度流
                                            └─ /uav/goal    (geometry_msgs/Point) home 目标点
                                                     │
                                          carlair_ros_bridge（唯一连仿真器的地方）
                                                     │
                                                  AirSim

为什么不像 gesture_control_drone 那样直连 AirSim
-----------------------------------------------
仓库里无人机模块统一走桥接层，桥接层负责：连接仿真器、限速/限幅、``cmd_timeout``
超时自动悬停、``/uav/odom`` 位姿发布、多机 ``vehicle_name``。本模块只做"手势指令
--> 桥接层消息"的翻译，不持有 AirSim 客户端，因此：

* 换仿真器（AirSim / CarlaAir / 多机）不用改本模块；
* 安全逻辑只有一份，不会出现"两套限速/两套超时"互相打架。

回调不阻塞
----------
订阅回调只入队；真正下发（速度流要按 ``1/stream_hz`` 节拍发 N 条，最长约 1 s）
在工作线程里做，避免堵住 ``rospy.spin()``。
"""
from __future__ import annotations

import os
import sys

# 与 gesture_control_drone 一致：catkin_install_python 在 devel 空间生成的是
# "启动器"，辅助模块靠 install(FILES) 装到同一目录，这里做一次 sys.path 引导。
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import queue
import threading
import time

import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import bridge_cmd as bc

NODE_NAME = "gesture_bridge_node"


def _p(name, default):
    return rospy.get_param("~" + name, default)


class GestureBridgeNode(object):
    """订阅手势指令，按桥接层契约翻译成动作/速度流/目标点。"""

    def __init__(self, publish=None, sleep=None, clock=None):
        rospy.init_node(NODE_NAME, anonymous=False)

        self.command_topic = _p("command_topic", "/gesture_command")
        self.cmd_vel_topic = _p("cmd_vel_topic", "/uav/cmd_vel")
        self.action_topic = _p("action_topic", "/uav/action")
        self.goal_topic = _p("goal_topic", "/uav/goal")
        self.status_topic = _p("status_topic", "/uav/status")
        self.odom_topic = _p("odom_topic", "/uav/odom")
        self.state_topic = _p("state_topic", "/gesture_bridge/state")

        self.speed = abs(float(_p("speed", 0.8)))
        self.yaw_rate = abs(float(_p("yaw_rate", 0.5)))
        self.action_duration = abs(float(_p("action_duration", 1.0)))
        self.stream_hz = float(_p("stream_hz", 20.0))
        self.home_altitude = abs(float(_p("home_altitude", 3.0)))
        self.estop_action = str(_p("estop_action", "hover")).lower()
        self.estop_zeros = max(1, int(_p("estop_zeros", 5)))
        self.cooldown = max(0.0, float(_p("command_cooldown", 0.5)))
        if self.stream_hz <= 0:
            raise ValueError("stream_hz 必须大于 0，收到 %r" % self.stream_hz)

        self._sleep = sleep if sleep is not None else time.sleep
        self._clock = clock if clock is not None else time.monotonic
        self._publish = publish if publish is not None else self._ros_publish

        self.pub_cmd = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.pub_action = rospy.Publisher(self.action_topic, String, queue_size=10)
        self.pub_goal = rospy.Publisher(self.goal_topic, Point, queue_size=10)
        self.pub_state = rospy.Publisher(self.state_topic, String, queue_size=10)

        self._queue = queue.Queue()
        self._stop = threading.Event()
        self._worker = None
        self._last_run = {}
        self.last_status = ""
        self.last_odom = None

        rospy.Subscriber(self.command_topic, String, self.on_command, queue_size=10)
        rospy.Subscriber(self.status_topic, String, self.on_status, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, self.on_odom, queue_size=10)
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("[bridge] 已启动：订阅 %s -> %s / %s / %s"
                      % (self.command_topic, self.cmd_vel_topic,
                         self.action_topic, self.goal_topic))
        rospy.loginfo("[bridge] 指令通道：%s" % bc.describe_map())
        rospy.loginfo("[bridge] 速度 %.2f m/s，偏航 %.2f rad/s，单次时长 %.2f s，"
                      "指令流 %.0f Hz（%d 条/次），急停=%s"
                      % (self.speed, self.yaw_rate, self.action_duration,
                         self.stream_hz,
                         bc.stream_count(self.action_duration, self.stream_hz),
                         self.estop_action))

    # ------------------------------------------------------------------ #
    # 回调（只记录/入队，不做耗时操作）
    # ------------------------------------------------------------------ #
    def on_command(self, msg):
        command = (msg.data or "").strip()
        if not bc.is_command(command):
            rospy.logwarn("[bridge] 收到未知指令 %r，已忽略（可用：%s）"
                          % (command, list(bc.ALL_COMMANDS)))
            return
        self._queue.put(command)

    def on_status(self, msg):
        self.last_status = (msg.data or "").strip()

    def on_odom(self, msg):
        try:
            self.last_odom = (msg.pose.pose.position.x,
                              msg.pose.pose.position.y,
                              msg.pose.pose.position.z)
        except Exception:            # pragma: no cover - 异常消息不该拖垮节点
            pass

    # ------------------------------------------------------------------ #
    # 发布
    # ------------------------------------------------------------------ #
    def _ros_publish(self, kind, payload):
        """把一条消息发出去；``kind`` ∈ {cmd, action, goal, state}。"""
        if kind == "cmd":
            vx, vy, vz, wz = payload
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.linear.z = vz
            twist.angular.z = wz
            self.pub_cmd.publish(twist)
        elif kind == "action":
            self.pub_action.publish(String(data=payload))
        elif kind == "goal":
            point = Point()
            point.x, point.y, point.z = payload
            self.pub_goal.publish(point)
        elif kind == "state":
            self.pub_state.publish(String(data=payload))
        else:                        # pragma: no cover - 编程错误
            raise ValueError("未知的发布类型 %r" % (kind,))

    def _set_state(self, text):
        self._publish("state", text)
        rospy.loginfo("[bridge] %s" % text)

    # ------------------------------------------------------------------ #
    # 执行
    # ------------------------------------------------------------------ #
    def handle_command(self, command, now=None):
        """执行一条指令，返回可读结果（失败返回 ``None``）。

        抽成独立方法，便于脱离 ROS 单测（注入 ``publish`` / ``sleep``）。
        """
        plan = bc.plan(command, speed=self.speed, yaw_rate=self.yaw_rate,
                       duration=self.action_duration, stream_hz=self.stream_hz,
                       home_altitude=self.home_altitude,
                       estop_action=self.estop_action,
                       estop_zeros=self.estop_zeros)
        if plan is None:
            rospy.logwarn("[bridge] 未知指令 %r" % (command,))
            return None

        now = self._clock() if now is None else now
        last = self._last_run.get(command)
        if last is not None and (now - last) < self.cooldown:
            rospy.logwarn_throttle(5.0, "[bridge] 指令 %s 处于冷却期（%.1fs），忽略"
                                   % (command, self.cooldown))
            return None
        self._last_run[command] = now

        period = 1.0 / self.stream_hz
        for index, velocity in enumerate(plan["velocities"]):
            self._publish("cmd", velocity)
            if index + 1 < len(plan["velocities"]):
                self._sleep(period)

        if plan["action"] is not None:
            self._publish("action", plan["action"])
        if plan["goal"] is not None:
            self._publish("goal", plan["goal"])

        summary = self._summarize(command, plan)
        self._set_state(summary)
        return summary

    @staticmethod
    def _summarize(command, plan):
        if plan["kind"] == bc.KIND_VELOCITY:
            vx, vy, vz, wz = plan["velocities"][0]
            return ("%s OK cmd_vel×%d (%.2f,%.2f,%.2f) yaw=%.2f rad/s"
                    % (command, len(plan["velocities"]), vx, vy, vz, wz))
        if plan["kind"] == bc.KIND_GOAL:
            return ("%s OK goal=(%.2f,%.2f,%.2f)"
                    % (command, plan["goal"][0], plan["goal"][1], plan["goal"][2]))
        if plan["kind"] == bc.KIND_ESTOP:
            return ("%s OK brake×%d + action=%s"
                    % (command, len(plan["velocities"]), plan["action"]))
        return "%s OK action=%s" % (command, plan["action"])

    def _run_worker(self):
        while not self._stop.is_set():
            try:
                command = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self.handle_command(command)
            except Exception as exc:          # 工作线程绝不因单条指令崩掉
                rospy.logerr("[bridge] 指令 %s 处理异常：%s" % (command, exc))
            finally:
                self._queue.task_done()

    def start_worker(self):
        self._worker = threading.Thread(target=self._run_worker,
                                        name="bridge_cmd_worker", daemon=True)
        self._worker.start()
        return self._worker

    # ------------------------------------------------------------------ #
    # 退出：先刹停再交还控制权
    # ------------------------------------------------------------------ #
    def shutdown(self):
        self._stop.set()
        if self._worker is not None and self._worker.is_alive():
            self._worker.join(timeout=2.0)
        try:
            # 把最后一帧速度指令顶成 0：桥接层 cmd_timeout 也会在 0.5 s 后悬停，
            # 但显式发零速更快、也更不容易让"最后一条正向速度"多跑 0.1 s。
            for _ in range(self.estop_zeros):
                self._publish("cmd", (0.0, 0.0, 0.0, 0.0))
            self._publish("action", "hover")
            rospy.loginfo("[bridge] 已退出：零速 + hover（桥接层 cmd_timeout 兜底）")
        except Exception as exc:              # pragma: no cover - 退出路径尽力而为
            rospy.logwarn("[bridge] 退出时下发安全指令失败：%s" % exc)


def main():
    node = GestureBridgeNode()
    node.start_worker()
    rospy.spin()
    node.shutdown()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
