#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""无人机控制节点：订阅 ``/gesture_command`` -> AirSim 执行 -> 状态回传。

职责
----
1. **执行**：订阅指令 token，交给 :class:`drone_client.AirSimDrone` 执行；
2. **回传**：按 ``state_rate`` 把载具状态发布成 ``nav_msgs/Odometry``（ENU，
   可直接在 RViz 里看），执行结果发布成 ``std_msgs/String``；
3. **保护**：指令去重冷却、未知指令拦截、碰撞告警、退出时先悬停再交还控制权。

设计要点（也是与早期版本的差别）
--------------------------------
* **回调不阻塞**：回调只入队，真正耗时的 AirSim 调用在工作线程里做
  （原实现直接在订阅回调里 ``takeoffAsync().join()``，而 ``rospy.spin()``
  是单线程的，起飞期间节点收不到任何消息）。
* **连接与指令超时**：都由 ``AirSimDrone`` 负责，失败会 ``logerr`` 并上报，
  不再只是静静地抛一条回调异常。
* **只认词表里的指令**：未知字符串直接忽略并告警。
"""
from __future__ import annotations

import os
import sys

# catkin_install_python 在 devel 空间给每个脚本生成的是"启动器"，
# 它只保证被启动的文件能跑，不保证同目录的辅助模块能被 import；
# 所以把本文件所在的源码目录放到 sys.path 最前面。
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import queue
import threading
import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import gesture_map as gm
from drone_client import AirSimDrone

NODE_NAME = "drone_control_node"


def _p(name, default):
    return rospy.get_param("~" + name, default)


class DroneControlNode(object):
    def __init__(self, drone=None):
        rospy.init_node(NODE_NAME, anonymous=False)

        self.command_topic = _p("command_topic", "/gesture_command")
        self.state_topic = _p("drone_state_topic", "/drone_state")
        self.odom_topic = _p("odom_topic", "/drone/odom")
        self.world_frame = _p("frame/world", "world")
        self.cooldown = max(0.0, float(_p("command_cooldown", 1.5)))
        self.state_rate = max(0.0, float(_p("state_rate", 5.0)))

        if drone is None:
            drone = AirSimDrone(
                ip=_p("airsim/ip", "127.0.0.1"),
                port=_p("airsim/port", 41451),
                vehicle_name=_p("airsim/vehicle_name", ""),
                estop_action=_p("estop_action", "hover"),
                speed=_p("speed", 1.0),
                move_duration=_p("move_duration", 1.0),
                takeoff_altitude=_p("takeoff_altitude", 3.0),
                max_speed=_p("max_speed", 3.0),
                max_altitude=_p("max_altitude", 30.0),
                min_altitude=_p("min_altitude", 0.5),
                yaw_rate=_p("yaw_rate", 30.0),
                command_timeout=_p("command_timeout", 10.0),
                takeoff_timeout=_p("takeoff_timeout", 60.0),
                land_timeout=_p("land_timeout", 60.0),
                state_lock_timeout=_p("state_lock_timeout", 0.1),
                connect_retries=_p("connect_retries", 3),
                log=rospy.loginfo)
        self.drone = drone

        self.pub_state = rospy.Publisher(self.state_topic, String, queue_size=10)
        self.pub_odom = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        rospy.Subscriber(self.command_topic, String, self.on_command, queue_size=10)

        self._queue = queue.Queue()
        self._stop = threading.Event()
        self._worker = None
        self._state_timer = None
        self._last_run = {}
        self._collision_warned = False
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("[drone] 已启动：订阅 %s，AirSim %s:%d，estop_action=%s，"
                      "起飞高度 %.1fm，限速 %.1f m/s，限高 %.1f m，指令冷却 %.1fs"
                      % (self.command_topic, self.drone.ip, self.drone.port,
                         self.drone.estop_action, self.drone.takeoff_altitude,
                         self.drone.max_speed, self.drone.max_altitude,
                         self.cooldown))
        rospy.loginfo("[drone] 支持指令：%s"
                      % "，".join("%s(%s)" % (c, gm.command_cn(c))
                                 for c in gm.ALL_COMMANDS))
        rospy.loginfo("[drone] 状态回传：%s（Odometry, %s 帧）与 %s（执行结果）"
                      % (self.odom_topic, self.world_frame, self.state_topic))

    # ------------------------------------------------------------------ #
    # 指令通道
    # ------------------------------------------------------------------ #
    def on_command(self, msg):
        """订阅回调：**只入队，不做任何耗时操作**。"""
        command = (msg.data or "").strip()
        if not gm.is_command(command):
            rospy.logwarn("[drone] 收到未知指令 %r，已忽略（可用：%s）"
                          % (command, list(gm.ALL_COMMANDS)))
            return
        self._queue.put(command)

    def handle_command(self, command, now=None):
        """执行一条指令并上报结果；返回是否执行成功（可离线单测）。"""
        if now is None:
            now = time.monotonic()
        last = self._last_run.get(command)
        if last is not None and (now - last) < self.cooldown:
            rospy.logwarn_throttle(5.0, "[drone] 指令 %s 处于冷却期（%.1fs），忽略"
                                   % (command, self.cooldown))
            return False

        ok = bool(self.drone.execute(command))
        self._last_run[command] = now
        if ok:
            n = self.drone.counts.get(command, 0) if hasattr(
                self.drone, "counts") else 0
            state = "%s OK" % command
            rospy.loginfo("[drone] 执行成功：%s（累计 %d 次）"
                          % (gm.command_cn(command), n))
        else:
            state = "%s FAIL %s" % (command, self.drone.last_error or "unknown")
            rospy.logerr("[drone] 执行失败：%s -> %s" % (command, state))
        self.pub_state.publish(String(data=state))
        return ok

    def _run_worker(self):
        while not self._stop.is_set():
            try:
                command = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self.handle_command(command)
            except Exception as exc:      # 工作线程绝不因单条指令崩掉
                rospy.logerr("[drone] 指令 %s 处理异常：%s" % (command, exc))
            finally:
                self._queue.task_done()

    def start_worker(self):
        self._worker = threading.Thread(target=self._run_worker,
                                        name="drone_cmd_worker", daemon=True)
        self._worker.start()
        return self._worker

    # ------------------------------------------------------------------ #
    # 状态回传
    # ------------------------------------------------------------------ #
    def publish_odom(self):
        """读一次载具状态并发成 ``nav_msgs/Odometry``（ENU）。

        未连接 / 读不到状态时**不发布**：否则会先发一帧 (0,0,0)，
        RViz 里表现为飞机从原点跳到真实位置。
        """
        state = self.drone.state()
        if not getattr(self.drone, "state_valid", True):
            return
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.world_frame
        msg.pose.pose.position.x = state.position_enu[0]
        msg.pose.pose.position.y = state.position_enu[1]
        msg.pose.pose.position.z = state.position_enu[2]
        msg.pose.pose.orientation.w = 1.0      # 暂不带偏航
        msg.twist.twist.linear.x = state.velocity_enu[0]
        msg.twist.twist.linear.y = state.velocity_enu[1]
        msg.twist.twist.linear.z = state.velocity_enu[2]
        self.pub_odom.publish(msg)

        if state.collision and not self._collision_warned:
            self._collision_warned = True
            rospy.logwarn("[drone] ⚠ 检测到碰撞：%s" % state.summary())
        return state

    def start_state_stream(self):
        """按 ``state_rate`` 起一个周期定时器持续发布状态。"""
        if self.state_rate <= 0:
            return None
        timer_cls = getattr(rospy, "Timer", None)
        duration_cls = getattr(rospy, "Duration", None)
        if timer_cls is None or duration_cls is None:
            return None
        self._state_timer = timer_cls(duration_cls(1.0 / self.state_rate),
                                      lambda _evt: self.publish_odom())
        return self._state_timer

    # ------------------------------------------------------------------ #
    def stop(self):
        self._stop.set()
        if self._worker is not None and self._worker.is_alive():
            self._worker.join(timeout=2.0)
        if self._state_timer is not None:
            try:
                self._state_timer.shutdown()
            except Exception:
                pass
            self._state_timer = None

    def shutdown(self):
        self.stop()
        try:
            self.drone.close()
        finally:
            rospy.loginfo("[drone] 已退出（执行统计：%s）"
                          % (getattr(self.drone, "counts", None) or "无"))


def main():
    node = DroneControlNode()
    node.start_worker()
    node.start_state_stream()
    rospy.spin()
    node.stop()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
